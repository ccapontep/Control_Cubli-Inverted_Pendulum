
% Create figure
figure('units','centimeters','outerposition',[0 0 35.34 29.82],'Renderer','OpenGL');

climax=400;   

ver = climax*[1 1 0;
    0 1 0;
    0 1 1;
    1 1 1;
    0 0 1;
    1 0 1;
    1 0 0;
    0 0 0];

%  Define the faces of the unit cubic
fac = [1 2 3 4;
    4 3 5 6;
    6 7 8 5;
    1 2 8 7;
    6 7 1 4;
    2 3 5 8];

% Set origin point for the cube in the form of [x,y,z]
origin = [0, 0, 0];



cube = patch('Faces',fac,'Vertices',ver,'FaceAlpha',0.75,'FaceColor',[.8 .8 .2]);

% specify patch (polygons) in patch() function
% cube=patch('Faces',fac,'Vertices',vert,'FaceAlpha',0.75,'FaceColor',[.8 .8 .2]);  % patch function
hold on


href = arrow3D([0,0,0],[0,0,900],[100/255 120/255 180/255],0.80);
% Body z-axis arrow blue
h3=arrow3D([0,0,0],[0,0,700],[31/255 120/255 180/255]);
% Body y-axis arrow green
h2=arrow3D([0,0,0],[0,700,0],[120/255 180/255 31/255]);
% Body x-axis arrow red
h1=arrow3D([0,0,0],[700,0,0],[180/255 31/255 120/255]);

lighting PHONG;

Ax = gca;

axis vis3d off;
rotate3d on;

% Axis properties
axis equal; %equal
axis on, grid on
view(100,5)


% camlight up
tsat=hgtransform('Parent', Ax);
set(cube,'Parent',tsat);
set(h1,'Parent',tsat);
set(h2,'Parent',tsat);
set(h3,'Parent',tsat);

axis([-700 700 -700 700 -100 1000])




 v = VideoWriter('UnderActuatedSecondCase.mp4','MPEG-4');
 open(v)


c=0;
% 
% while(true)
for i= 1 : length(tspan)-1
    
   % Actual quaternion
    quat=[p(i+1,13);p(i+1,10);p(i+1,11);p(i+1,12)];
    
    % Calculate the actual Euler axis and angle
    theta=2*acos(quat(1));
    epsnorm=sqrt(quat(2)^2+quat(3)^2+quat(4)^2);
    if epsnorm==0
        eps1=0;eps2=0;eps3=1;
    else
        eps1=quat(2)/epsnorm;
        eps2=quat(3)/epsnorm;
        eps3=quat(4)/epsnorm;
    end
    
    % Create satellite graphic translation (current position)
%    translation=makehgtform('translate',trans);
  
    % Create Cubli graphic eigenaxis rotation
    eigenaxisrotation=makehgtform('axisrotate',[eps1,eps2,eps3],theta);

    % Put Cubli the new position
    set(tsat,'matrix',eigenaxisrotation*eye(4));
   

 
   frame = getframe(gcf);
   writeVideo(v,frame);
   
   pause(0.1)
   c=c+1;

end

pause(2)
close(v);
% end
