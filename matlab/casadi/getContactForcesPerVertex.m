function F_contact = getContactForcesPerVertex(t_steps, Fx, Fy, Fz, Tz, xCoP, yCoP, lx, ly, mu)

import casadi.*

opti = casadi.Opti();

f = opti.variable(12,t_steps);
fx = f(1:3:10,:);
fy = f(2:3:11,:);
fz = f(3:3:12,:);

cost = 0;
for j=1:t_steps

    F_contact = reshape(f(:,j),3,4);
    a = lx/2-xCoP(j);
    b = lx/2+xCoP(j); 
    c = ly/2-yCoP(j);
    d = ly/2+yCoP(j);

    if a == 0 || c == 0
        error("COP is on one of the foot edges")
    end

    for i = 1:4
      
       cost = cost + 0.5*sumsqr(mean(F_contact,2) - ...
           [fx(i,j);fy(i,j);fz(i,j)]);
    end

    opti.subject_to( fx(1,j)>=-mu*fz(1,j))
    opti.subject_to( fx(2,j)>=-mu*fz(2,j))
    opti.subject_to( fx(3,j)>=-mu*fz(3,j))
    opti.subject_to( fx(4,j)>=-mu*fz(4,j))
    opti.subject_to( fx(1,j)<=mu*fz(1,j))
    opti.subject_to( fx(2,j)<=mu*fz(2,j))
    opti.subject_to( fx(3,j)<=mu*fz(3,j))
    opti.subject_to( fx(4,j)<=mu*fz(4,j))
    opti.subject_to( (fx(1,j)+fx(2,j)+fx(3,j)+fx(4,j)) == Fx(j) );

    opti.subject_to( fy(1,j)>=-mu*fz(1,j))
    opti.subject_to( fy(2,j)>=-mu*fz(2,j))
    opti.subject_to( fy(3,j)>=-mu*fz(3,j))
    opti.subject_to( fy(4,j)>=-mu*fz(4,j))    
    opti.subject_to( fy(1,j)<=mu*fz(1,j))
    opti.subject_to( fy(2,j)<=mu*fz(2,j))
    opti.subject_to( fy(3,j)<=mu*fz(3,j))
    opti.subject_to( fy(4,j)<=mu*fz(4,j))
    opti.subject_to( (fy(1,j)+fy(2,j)+fy(3,j)+fy(4,j)) == Fy(j) );
    
    opti.subject_to( (fx(1,j)+fx(4,j))*c - ...
                     (fx(3,j)+fx(2,j))*d +...
                     (fy(1,j)+fy(2,j))*a - ...
                     (fy(3,j)+fy(4,j))*b == Tz(j) );
    
    opti.subject_to( fz(1,j)>0 );
    opti.subject_to( fz(2,j)>0 );
    opti.subject_to( fz(3,j)>0 );
    opti.subject_to( fz(4,j)>0 );

    opti.subject_to( (fz(1,j)+fz(2,j)) == b/a*(fz(3,j)+fz(4,j)) );
    opti.subject_to( (fz(1,j)+fz(4,j)) == d/c*(fz(2,j)+fz(3,j)) );
    opti.subject_to( (fz(1,j)+fz(2,j)+fz(3,j)+fz(4,j)) == Fz(j) );

end

opti.minimize( cost );
opti.solver('ipopt');
sol = opti.solve();

F_contact = sol.value(f);
