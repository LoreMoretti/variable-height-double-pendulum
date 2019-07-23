#include <Responder.h>
#include <cmath>
#include <cassert>

using std::placeholders::_1;

bool StepUpPlanner::Responder::prepareSolver(const controller_msgs::msg::StepUpPlannerRequestMessage::SharedPtr msg,
                                             StepUpPlanner::State &initialState, StepUpPlanner::References &references)
{
    if (!m_solver.isReady()) {
        sendErrorMessage(Errors::PARAMETERS_NOT_SET,
                         "Parameters were not specified.");
        return false;
    }

    if (msg->phases.size() != m_solver.numberOfPhases()) {
        sendErrorMessage(Errors::REQUEST_ERROR,
                         "The number of phases is different from the one specified in the parameters message");
        return false;
    }

    for (size_t p = 0; p < m_solver.numberOfPhases(); ++p) {
        StepUpPlanner::Phase& phase = m_solver.getPhase(p);
        if ((phase.getPhaseType() == StepUpPlanner::PhaseType::SINGLE_SUPPORT_LEFT) ||
            (phase.getPhaseType() == StepUpPlanner::PhaseType::DOUBLE_SUPPORT)) {
            auto& leftPosition = msg->phases[p].left_foot_pose.position;
            phase.setLeftPosition(leftPosition.x, leftPosition.y, leftPosition.z);
            auto& leftQuaternion = msg->phases[p].left_foot_pose.orientation;
            phase.leftRotation().setFromQuaternion(leftQuaternion.w, leftQuaternion.x, leftQuaternion.y, leftQuaternion.z);
        }

        if ((phase.getPhaseType() == StepUpPlanner::PhaseType::SINGLE_SUPPORT_RIGHT) ||
            (phase.getPhaseType() == StepUpPlanner::PhaseType::DOUBLE_SUPPORT)) {
            auto& rightPosition = msg->phases[p].right_foot_pose.position;
            phase.setRightPosition(rightPosition.x, rightPosition.y, rightPosition.z);
            auto& rightQuaternion = msg->phases[p].right_foot_pose.orientation;
            phase.rightRotation().setFromQuaternion(rightQuaternion.w, rightQuaternion.x, rightQuaternion.y, rightQuaternion.z);
        }

        phase.setDurationSettings(msg->phases[p].minimum_duration, msg->phases[p].maximum_duration, msg->phases[p].desired_duration);
    }

    initialState.setPosition(msg->initial_com_position.x, msg->initial_com_position.y, msg->initial_com_position.z);
    initialState.setVelocity(msg->initial_com_velocity.x, msg->initial_com_velocity.y, msg->initial_com_velocity.z);

    references.zero();
    references.desiredState().setPosition(msg->desired_com_position.x, msg->desired_com_position.y, msg->desired_com_position.z);
    references.desiredState().setVelocity(msg->desired_com_velocity.x, msg->desired_com_velocity.y, msg->desired_com_velocity.z);

    references.desiredControl().left().setMultiplier(msg->left_desired_final_control.multiplier);
    references.desiredControl().left().setCoP(msg->left_desired_final_control.cop.x, msg->left_desired_final_control.cop.y);
    references.desiredControl().right().setMultiplier(msg->right_desired_final_control.multiplier);
    references.desiredControl().right().setCoP(msg->right_desired_final_control.cop.x, msg->right_desired_final_control.cop.y);

    bool ok = references.setDesiredLegLength(msg->desired_leg_length);
    if (!ok) {
        sendErrorMessage(Errors::REQUEST_ERROR, "The desired leg length is not supposed to be smaller than 0.");
        return false;
    }

    return true;
}

void StepUpPlanner::Responder::respond(const controller_msgs::msg::StepUpPlannerRequestMessage::SharedPtr msg)
{

    StepUpPlanner::State initialState;
    StepUpPlanner::References references;

    if (!prepareSolver(msg, initialState, references)) {
        return;
    }

    bool ok = m_solver.solve(initialState, references);
    if (!ok) {
        sendErrorMessage(Errors::SOLVER_ERROR, "Failed to solve the optimization problem.");
        return;
    }

    ok = m_solver.getFullSolution(m_phases);
    assert(ok);

    prepareRespondMessage();

    sendCenterOfMassTrajectoryMessages();

    sendFootStepDataListMessage();

    m_respondPublisher->publish(m_respondMessage);
}

bool StepUpPlanner::Responder::processPhaseSettings(const controller_msgs::msg::StepUpPlannerParametersMessage::SharedPtr msg)
{
    m_phases.resize(msg->phases_settings.size());

    for (size_t phase = 0; phase < m_phases.size(); ++phase) {
        StepUpPlanner::Step leftStep, *leftPointer, rightStep, *rightPointer;

        auto& setting = msg->phases_settings[phase];
        auto& leftParameters = setting.left_step_parameters;
        auto& rightParameters = setting.right_step_parameters;

        if (leftParameters.state == leftParameters.STAND) {

            leftPointer = &leftStep;

            std::vector<StepUpPlanner::Vertex> vertices;
            vertices.resize(leftParameters.foot_vertices.size());
            for (size_t vertex = 0; vertex < leftParameters.foot_vertices.size(); ++vertex) {
                vertices[vertex].x = leftParameters.foot_vertices[vertex].x;
                vertices[vertex].y = leftParameters.foot_vertices[vertex].y;
            }

            bool ok = leftStep.setVertices(vertices);

            if (!ok) {
                sendErrorMessage(Errors::PARAMETERS_ERROR,
                                 "Failed to set the vertices for the left foot in the phase with index" + std::to_string(phase));
                return false;
            }

        } else {
            leftPointer = nullptr;
        }

        if (setting.right_step_parameters.state == setting.right_step_parameters.STAND) {
            rightPointer = &rightStep;

            std::vector<StepUpPlanner::Vertex> vertices;
            vertices.resize(rightParameters.foot_vertices.size());
            for (size_t vertex = 0; vertex < rightParameters.foot_vertices.size(); ++vertex) {
                vertices[vertex].x = rightParameters.foot_vertices[vertex].x;
                vertices[vertex].y = rightParameters.foot_vertices[vertex].y;
            }

            bool ok = rightStep.setVertices(vertices);

            if (!ok) {
                sendErrorMessage(Errors::PARAMETERS_ERROR,
                                 "Failed to set the vertices for the right foot in the phase with index" + std::to_string(phase));
                return false;
            }

        } else {
            rightPointer = nullptr;
        }

        m_phases[phase] = StepUpPlanner::Phase(leftPointer, rightPointer);
    }

    return true;
}

bool StepUpPlanner::Responder::processPlannerSettings(const controller_msgs::msg::StepUpPlannerParametersMessage::SharedPtr msg,
                                                      StepUpPlanner::Settings& settings)
{
    settings.phaseLength() = msg->phase_length;
    settings.solverVerbosity() = msg->solver_verbosity;

    bool ok = settings.setMaximumLegLength(msg->max_leg_length);
    if (!ok) {
        sendErrorMessage(Errors::PARAMETERS_ERROR,
                         "The max_leg_length is supposed to be a positive number");
        return false;
    }

    if (msg->ipopt_linear_solver.size()) {
        ok = settings.setIpoptLinearSolver(msg->ipopt_linear_solver);
        if (!ok) {
            sendErrorMessage(Errors::PARAMETERS_ERROR,
                             "Unknown solver name. "
                             "See options at https://www.coin-or.org/Ipopt/documentation/node51.html#SECTION0001111010000000000000.");
            return false;
        }
    }

    ok = settings.setFinalStateAnticipation(msg->final_state_anticipation);
    if (!ok) {
        sendErrorMessage(Errors::PARAMETERS_ERROR,
                         "The final state anticipation is supposed to be in the interval [0.0, 1.0].");
        return false;
    }

    settings.setStaticFrictionCoefficient(msg->static_friction_coefficient);
    if (!ok) {
        sendErrorMessage(Errors::PARAMETERS_ERROR,
                         "The staticFrictionCoeff is supposed to be a positive number.");
        return false;
    }

    settings.setTorsionalFrictionCoefficient(msg->torsional_friction_coefficient);
    if (!ok) {
        sendErrorMessage(Errors::PARAMETERS_ERROR,
                         "The torsionalFrictionCoeff is supposed to be a positive number.");
        return false;
    }

    StepUpPlanner::CostWeights& weights = settings.costWeights();
    auto& messageWeights = msg->cost_weights;
    weights.cop = messageWeights.cop;
    weights.torques = messageWeights.torques;
    weights.multipliers = messageWeights.control_multipliers;
    weights.finalControl = messageWeights.final_control;
    weights.maxMultiplier = messageWeights.max_control_multiplier;
    weights.finalStateError = messageWeights.final_state;
    weights.controlVariations = messageWeights.control_variations;
    weights.durationsDifference = messageWeights.durations_difference;

    return true;
}

void StepUpPlanner::Responder::processParameters(const controller_msgs::msg::StepUpPlannerParametersMessage::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "[processParameters] Parameters received");

    if (!processPhaseSettings(msg)) {
        return;
    }

    StepUpPlanner::Settings settings;

    if (!processPlannerSettings(msg, settings)) {
        return;
    }

    bool ok = m_solver.resetProblem(m_phases, settings);
    if (!ok) {
        sendErrorMessage(Errors::PARAMETERS_ERROR,
                         "Failed to reset the problem.");
        return;
    }

    if (msg->send_com_messages || msg->include_com_messages) {

        if (msg->send_com_messages) {
            m_CoMMessagePublisher = this->create_publisher<controller_msgs::msg::CenterOfMassTrajectoryMessage>(msg->com_messages_topic);
        } else {
            m_CoMMessagePublisher = nullptr;
        }
        if (msg->max_com_message_length == 0) {
            sendErrorMessage(Errors::PARAMETERS_ERROR,
                             "The max_message_length is supposed to be a positive number.");
            return;
        }

        m_CoMMessagesPerPhase = static_cast<size_t>(std::ceil(settings.phaseLength() / static_cast<double>(msg->max_com_message_length)));
        assert(m_CoMMessagesPerPhase > 0);

        m_CoMMessages.clear();

        for (size_t i = 0; i < m_phases.size() * m_CoMMessagesPerPhase; ++i) {
            m_CoMMessages.push_back(std::make_shared<controller_msgs::msg::CenterOfMassTrajectoryMessage>());
        }

        if (msg->include_com_messages) {
            m_respondMessage->com_messages.resize(m_CoMMessages.size());
        } else {
            m_respondMessage->com_messages.clear();
        }

    } else {
        m_CoMMessagePublisher = nullptr;
        m_respondMessage->com_messages.clear();
    }

    if (msg->send_footstep_messages || msg->include_footstep_messages) {

        if (m_phases.size() < 2) {
            sendErrorMessage(Errors::PARAMETERS_ERROR,
                             "Cannot send feet message if the phases are less than 2.");
            return;
        }

        for (size_t i = 1; i < m_phases.size(); ++i) {

            if (m_phases[i].getPhaseType() == m_phases[i-1].getPhaseType()) {
                sendErrorMessage(Errors::PARAMETERS_ERROR,
                                 "Cannot send feet message if two consecutive phases are equal.");
                return;
            }
        }

        if (msg->send_footstep_messages) {
            m_feetMessagePublisher = this->create_publisher<controller_msgs::msg::FootstepDataListMessage>(msg->footstep_messages_topic);
        } else {
            m_feetMessagePublisher = nullptr;
        }

        m_feetMessage = std::make_shared<controller_msgs::msg::FootstepDataListMessage>();

        controller_msgs::msg::FootstepDataMessage templateStep;
        templateStep.transfer_duration = 0.0;

        size_t numberOfSingleSupports = 0;
        for (size_t i = 0; i < m_phases.size() - 1; ++i) {
            if ((m_phases[i].getPhaseType() == StepUpPlanner::PhaseType::SINGLE_SUPPORT_LEFT) ||
                (m_phases[i].getPhaseType() == StepUpPlanner::PhaseType::SINGLE_SUPPORT_RIGHT)) {
                numberOfSingleSupports++;
            }
        }

        if (numberOfSingleSupports == 0) {
            sendErrorMessage(Errors::PARAMETERS_ERROR,
                             "No footstep message can be sent given the desired walking phases.");
            return;
        }

        m_feetMessage->footstep_data_list.resize(numberOfSingleSupports, templateStep); //Initializing to zero all the transfer times

        if (msg->include_footstep_messages) {
            m_respondMessage->foostep_messages.resize(1);
        } else {
            m_respondMessage->foostep_messages.clear();
        }

    } else {
        m_feetMessagePublisher = nullptr;
        m_respondMessage->foostep_messages.clear();
    }

    RCLCPP_INFO(this->get_logger(), "[processParameters] Parameters set correctly.");

    ackReceivedParameters(msg->sequence_id);

}

void StepUpPlanner::Responder::sendErrorMessage(StepUpPlanner::Responder::Errors errorType, const std::string &errorMessage)
{
    auto error = std::make_shared<controller_msgs::msg::StepUpPlannerErrorMessage>();

    if (errorType == Errors::PARAMETERS_ERROR) {
        error->error_code = 1;
    } else if (errorType == Errors::PARAMETERS_NOT_SET) {
        error->error_code = 2;
    } else if (errorType == Errors::REQUEST_ERROR) {
        error->error_code = 3;
    } else if (errorType == Errors::SOLVER_ERROR) {
        error->error_code = 4;
    }

    error->error_description = errorMessage;

    m_errorPublisher->publish(error);

    RCLCPP_ERROR(this->get_logger(), "[StepUpPlanner::Responder] " + errorMessage);
}

void StepUpPlanner::Responder::prepareRespondMessage()
{
    m_respondMessage->phases_result.resize(m_phases.size());

    for (size_t p = 0; p < m_phases.size(); ++p) {
        auto& phaseResult = m_respondMessage->phases_result[p];

        phaseResult.duration = static_cast<double>(m_phases[p].duration());

        phaseResult.com_position.resize(m_phases[p].states().size());
        phaseResult.com_velocity.resize(m_phases[p].states().size());

        for (size_t s = 0; s < m_phases[p].states().size(); ++s) {
            phaseResult.com_position[s].x = m_phases[p].states()[s].position(0);
            phaseResult.com_position[s].y = m_phases[p].states()[s].position(1);
            phaseResult.com_position[s].z = m_phases[p].states()[s].position(2);

            phaseResult.com_velocity[s].x = m_phases[p].states()[s].velocity(0);
            phaseResult.com_velocity[s].y = m_phases[p].states()[s].velocity(1);
            phaseResult.com_velocity[s].z = m_phases[p].states()[s].velocity(2);
        }

        phaseResult.com_acceleration.resize(m_phases[p].controls().size());
        phaseResult.left_controls.resize(m_phases[p].controls().size());
        phaseResult.right_controls.resize(m_phases[p].controls().size());

        for (size_t c = 0; c < m_phases[p].controls().size(); ++c) {

            phaseResult.com_acceleration[c].x = m_phases[p].controls()[c].acceleration(0);
            phaseResult.com_acceleration[c].y = m_phases[p].controls()[c].acceleration(1);
            phaseResult.com_acceleration[c].z = m_phases[p].controls()[c].acceleration(2);

            phaseResult.left_controls[c].multiplier = static_cast<double>(m_phases[p].controls()[c].left().multiplier());
            phaseResult.left_controls[c].cop.x = m_phases[p].controls()[c].left().cop(0);
            phaseResult.left_controls[c].cop.y = m_phases[p].controls()[c].left().cop(1);

            phaseResult.right_controls[c].multiplier = static_cast<double>(m_phases[p].controls()[c].right().multiplier());
            phaseResult.right_controls[c].cop.x = m_phases[p].controls()[c].right().cop(0);
            phaseResult.right_controls[c].cop.y = m_phases[p].controls()[c].right().cop(1);
        }
    }

}

void StepUpPlanner::Responder::sendCenterOfMassTrajectoryMessages()
{
    if (!m_CoMMessagePublisher && !m_respondMessage->com_messages.size()) {
        return;
    }

    double time = 0.0;
    for (size_t p = 0; p < m_phases.size(); ++p) {
        size_t phaseLength = m_phases[p].states().size();
        double dT = static_cast<double>(m_phases[p].duration()) / phaseLength;
        size_t pointIndex = 0;

        for (size_t m = 0; m < m_CoMMessagesPerPhase; ++m) {
            size_t messagesElements;
            if (m == m_CoMMessagesPerPhase - 1) {
                messagesElements = phaseLength - (m_CoMMessagesPerPhase - 1) * phaseLength / m_CoMMessagesPerPhase; //account for roundings
            } else {
                messagesElements = phaseLength / m_CoMMessagesPerPhase;
            }

            size_t messageIndex = p * m_CoMMessagesPerPhase + m;

            auto& euclideanTrajectory = m_CoMMessages[messageIndex]->euclidean_trajectory;

            euclideanTrajectory.queueing_properties.sequence_id = static_cast<unsigned int>(messageIndex);
            euclideanTrajectory.queueing_properties.message_id = static_cast<long>(messageIndex);

            if (messageIndex == 0) {
                euclideanTrajectory.queueing_properties.set__execution_mode(
                    euclideanTrajectory.queueing_properties.EXECUTION_MODE_OVERRIDE);
            } else {
                euclideanTrajectory.queueing_properties.set__execution_mode(euclideanTrajectory.queueing_properties.EXECUTION_MODE_QUEUE);
                euclideanTrajectory.queueing_properties.set__previous_message_id(static_cast<long>(messageIndex - 1));
            }

            euclideanTrajectory.taskspace_trajectory_points.resize(messagesElements);

            for (size_t i = 0; i < messagesElements; ++i) {
                auto& point = euclideanTrajectory.taskspace_trajectory_points[i];
                point.time = time;
                point.position.x = m_phases[p].states()[pointIndex].position(0);
                point.position.y = m_phases[p].states()[pointIndex].position(1);
                point.position.z = m_phases[p].states()[pointIndex].position(2);

                point.linear_velocity.x = m_phases[p].states()[pointIndex].velocity(0);
                point.linear_velocity.y = m_phases[p].states()[pointIndex].velocity(1);
                point.linear_velocity.z = m_phases[p].states()[pointIndex].velocity(2);

                time += dT;
                pointIndex++;
            }

            time = dT; //The time for each message is relative to the previous queued message.

            if (m_CoMMessagePublisher) {
                m_CoMMessagePublisher->publish(m_CoMMessages[messageIndex]);
            }

            if (m_respondMessage->com_messages.size()) {
                m_respondMessage->com_messages[messageIndex] = *m_CoMMessages[messageIndex];
            }
        }
        assert(pointIndex == phaseLength);
    }
}

void StepUpPlanner::Responder::sendFootStepDataListMessage()
{
    if (!m_feetMessagePublisher && !m_respondMessage->foostep_messages.size()) {
        return;
    }

    if (m_phases.back().getPhaseType() == StepUpPlanner::PhaseType::DOUBLE_SUPPORT) {
        m_feetMessage->final_transfer_duration = static_cast<double>(m_phases.back().duration());
    } else {
        m_feetMessage->final_transfer_duration = 1e-4;
    }

    size_t stepIndex = 0;
    size_t i = 0;
    while (i < m_phases.size() - 1) {
        if (m_phases[i].getPhaseType() == StepUpPlanner::PhaseType::DOUBLE_SUPPORT) {
            m_feetMessage->footstep_data_list[stepIndex].transfer_duration = static_cast<double>(m_phases[i].duration());
            ++i;
        } else if (m_phases[i].getPhaseType() == StepUpPlanner::PhaseType::SINGLE_SUPPORT_LEFT) {
            m_feetMessage->footstep_data_list[stepIndex].swing_duration = static_cast<double>(m_phases[i].duration());
            ++i;
            m_feetMessage->footstep_data_list[stepIndex].robot_side = m_feetMessage->footstep_data_list[stepIndex].ROBOT_SIDE_LEFT;
            m_feetMessage->footstep_data_list[stepIndex].location.x = m_phases[i].leftPosition(0);
            m_feetMessage->footstep_data_list[stepIndex].location.y = m_phases[i].leftPosition(1);
            m_feetMessage->footstep_data_list[stepIndex].location.z = m_phases[i].leftPosition(2);

            m_feetMessage->footstep_data_list[stepIndex].orientation.w = m_phases[i].leftRotation().asQuaternion(0);
            m_feetMessage->footstep_data_list[stepIndex].orientation.x = m_phases[i].leftRotation().asQuaternion(1);
            m_feetMessage->footstep_data_list[stepIndex].orientation.y = m_phases[i].leftRotation().asQuaternion(2);
            m_feetMessage->footstep_data_list[stepIndex].orientation.z = m_phases[i].leftRotation().asQuaternion(3);

            m_feetMessage->footstep_data_list[stepIndex].predicted_contact_points_2d.resize(m_phases[i].getLeftStep().getVertices().size());
            for (size_t v = 0; v < m_phases[i].getLeftStep().getVertices().size(); ++v) {
                const StepUpPlanner::Vertex& vertex = m_phases[i].getLeftStep().getVertices()[v];

                m_feetMessage->footstep_data_list[stepIndex].predicted_contact_points_2d[v].x = vertex.x;
                m_feetMessage->footstep_data_list[stepIndex].predicted_contact_points_2d[v].y = vertex.y;
            }

            stepIndex++;
        } else if (m_phases[i].getPhaseType() == StepUpPlanner::PhaseType::SINGLE_SUPPORT_RIGHT) {
            m_feetMessage->footstep_data_list[stepIndex].swing_duration = static_cast<double>(m_phases[i].duration());
            ++i;
            m_feetMessage->footstep_data_list[stepIndex].robot_side = m_feetMessage->footstep_data_list[stepIndex].ROBOT_SIDE_RIGHT;
            m_feetMessage->footstep_data_list[stepIndex].location.x = m_phases[i].rightPosition(0);
            m_feetMessage->footstep_data_list[stepIndex].location.y = m_phases[i].rightPosition(1);
            m_feetMessage->footstep_data_list[stepIndex].location.z = m_phases[i].rightPosition(2);

            m_feetMessage->footstep_data_list[stepIndex].orientation.w = m_phases[i].rightRotation().asQuaternion(0);
            m_feetMessage->footstep_data_list[stepIndex].orientation.x = m_phases[i].rightRotation().asQuaternion(1);
            m_feetMessage->footstep_data_list[stepIndex].orientation.y = m_phases[i].rightRotation().asQuaternion(2);
            m_feetMessage->footstep_data_list[stepIndex].orientation.z = m_phases[i].rightRotation().asQuaternion(3);

            m_feetMessage->footstep_data_list[stepIndex].predicted_contact_points_2d.resize(
                m_phases[i].getRightStep().getVertices().size());
            for (size_t v = 0; v < m_phases[i].getRightStep().getVertices().size(); ++v) {
                const StepUpPlanner::Vertex& vertex = m_phases[i].getRightStep().getVertices()[v];

                m_feetMessage->footstep_data_list[stepIndex].predicted_contact_points_2d[v].x = vertex.x;
                m_feetMessage->footstep_data_list[stepIndex].predicted_contact_points_2d[v].y = vertex.y;
            }

            stepIndex++;
        }
    }

    assert(stepIndex == m_feetMessage->footstep_data_list.size());

    if (m_feetMessagePublisher) {
        m_feetMessagePublisher->publish(m_feetMessage);
    }

    if (m_respondMessage->foostep_messages.size()) {
        m_respondMessage->foostep_messages[0] = *m_feetMessage;
    }
}

void StepUpPlanner::Responder::ackReceivedParameters(unsigned int message_id)
{
    auto error = std::make_shared<controller_msgs::msg::StepUpPlannerErrorMessage>();
    error->error_description = "";
    error->error_code = 0;
    error->sequence_id_received = message_id;
    m_errorPublisher->publish(error);
}

StepUpPlanner::Responder::Responder()
: Node("StepUpPlannerResponder")
{
    m_respondMessage = std::make_shared<controller_msgs::msg::StepUpPlannerRespondMessage>();
    m_respondPublisher = this->create_publisher<controller_msgs::msg::StepUpPlannerRespondMessage>(STEPUPPLANNER_RESPOND_TOPIC);
    m_errorPublisher = this->create_publisher<controller_msgs::msg::StepUpPlannerErrorMessage>(STEPUPPLANNER_ERRORS_TOPIC);
    m_requestSubscriber = this->create_subscription<controller_msgs::msg::StepUpPlannerRequestMessage>(
        STEPUPPLANNER_REQUEST_TOPIC, std::bind(&StepUpPlanner::Responder::respond, this, _1));
    m_parametersSubscriber = this->create_subscription<controller_msgs::msg::StepUpPlannerParametersMessage>(
        STEPUPPLANNER_PARAMETERS_TOPIC, std::bind(&StepUpPlanner::Responder::processParameters, this, _1));

    m_CoMMessagePublisher = nullptr;
    m_feetMessagePublisher = nullptr;
}

