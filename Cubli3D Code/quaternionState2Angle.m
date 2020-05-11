function angles = quaternionState2Angle(p)

   %note you MUST check where are located the quaternions and in witch
   %order
   
    angles = zeros(length(p(:,10)),3);

    for i = 1 :length(p(:,10))

       %[roll, pitch, yaw] = quat2angle([p(i,10),p(i,11),p(i,12),p(i,13)],'ZXY');
          roll = atan2d(norm(cross([1 0 0],-p(i,1:3))),dot([1 0 0],-p(i,1:3)));
         pitch = atan2d(norm(cross([0 1 0],-p(i,1:3))),dot([0 1 0],-p(i,1:3)));
           yaw = atan2d(norm(cross([0 0 1],-p(i,1:3))),dot([0 0 1],-p(i,1:3)));
         angles(i,1:3) = [roll, pitch, yaw];

    end


end