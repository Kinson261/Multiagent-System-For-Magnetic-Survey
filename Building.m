%Buildings generates vertical columns at random positions and dimensions.
M=occupancyMap3D;
%shuffle random seed.
rng shuffle
mapseed=rng;

%Set occupancy of the ground plane.
for i=0:1:200
    for j=0:1:200
        M.setOccupancy([i,j,0],1);
    end
end

for columns=1:1:5
    [pos,w,l,h]=randomColumn();
    while(pos(1)+w>200)||(pos(2)+l>200)
        [pos,w,l,h]=randomColumn();
    end
    for i=0:1:w
        for j=0:1:l
            for k=0:1:h
                %Ensure pos to start from Z 0
                M.setOccupancy([pos(1,1)+i, pos(2,1)+j,pos(3,1)+k],1);
            end
        end
    end
    %show occupancy map.
    M.show();
end

%%

%load map
omap = M;
omap.FreeThreshold = omap.OccupiedThreshold;
inflate(omap,1);
% figure("Name","CityBlock");
% show(omap);
hmap = show(omap);
hold on;

%set start and goal pose
iter = 1;

startPose = [20 30 90 pi/2];
%startPose = input("StartPose = [x y z teta] ? ");

dx = 50;
dy = 120;

for j = 1:8
    if  rem(j,5) == 1
        goalPose = [startPose(1,1) startPose(1,2)+dy startPose(1,3) pi/2];
    elseif rem(j,5) == 2
        startPose = [startPose(1,1) startPose(1,2)+dy startPose(1,3) pi/2];
        goalPose = [startPose(1,1)+dx startPose(1,2) startPose(1,3) -pi/2];
    elseif rem(j,5) == 3
        startPose = [startPose(1,1)+dx startPose(1,2) startPose(1,3) -pi/2];
        goalPose = [startPose(1,1) startPose(1,2)-dy startPose(1,3) -pi/2];
    elseif rem(j,5) == 4
        startPose = [startPose(1,1) startPose(1,2)-dy startPose(1,3) -pi/2];
        goalPose = [startPose(1,1)+dx startPose(1,2) startPose(1,3) pi/2];
    elseif rem(j,5) == 0
        startPose = [startPose(1,1)+dx startPose(1,2) startPose(1,3) pi/2];
        goalPose = [startPose(1,1) startPose(1,2)+dy startPose(1,3) pi/2];
    end


    %     scatter3(hMap, startPose(1), startPose(2), startPose(3), 50, "red","filled");
    %     scatter3(hMap, goalPose(1), goalPose(2), goalPose(3), 50, "green","filled");
    %     view  ([-31 63]);
    %     hold on

    %Plan a path w/ RRT using 3D Dubbins motion primitives
    % Define the State Space object
    ss = ExampleHelperUAVStateSpace("MaxRollAngle",pi/20,"AirSpeed",3,"FlightPathAngleLimit",[-1 1],"Bounds",[0 300; 0 300; 0 100; -pi pi]);
    threshold = [(goalPose+0.5)' (goalPose-0.5)'; -2*pi 2*pi];
    setWorkspaceGoalRegion(ss, goalPose, threshold);

    %define the State Validator Object
    sv = validatorOccupancyMap3D(ss,"Map",omap);
    sv.ValidationDistance = 0.1;

    %Set up the RRT path planner
    planner = plannerRRTStar(ss,sv);
    planner.ContinueAfterGoalReached = true;
    planner.MaxConnectionDistance = 300;
    planner.GoalBias = 0.5;
    planner.MaxIterations = 3000
    planner.GoalReachedFcn = @(~, x, y)(norm(x(1:3)-y(1:3))<1);

    %Execute path planning
    [pthObj,solnInfo] = plan(planner,startPose,goalPose);

    h1Map = show(omap);
    hold on;

    % Simulate a UAV following the planned path
    if (solnInfo.IsPathFound)
        j
        startPose
        goalPose

        % Visualize the 3-D map
        %         h1Map = show(omap);
        %         hold on;
        scatter3(hmap, startPose(1),startPose(2),startPose(3),50,"red","filled");
        scatter3(hmap, goalPose(1),goalPose(2),goalPose(3),50,"green","filled");
        interpolatedPathObj = copy(pthObj);
        interpolate(interpolatedPathObj,1000);


        % Plot the interpolated path based on UAV Dubins connections
        hReference = plot3(interpolatedPathObj.States(:,1), ...
            interpolatedPathObj.States(:,2), ...
            interpolatedPathObj.States(:,3), ...
            "LineWidth",2,"Color","g");


        % Plot simulated UAV trajectory based on fixed-wing guidance model
        % Compute total time of flight and add a buffer
        timeToReachGoal = 1.05*pathLength(pthObj)/ss.AirSpeed;
        waypoints = interpolatedPathObj.States;
        [xENU,yENU,zENU] = exampleHelperSimulateUAV(waypoints,ss.AirSpeed,timeToReachGoal);
        %hSimulated = plot3(xENU,yENU,zENU,"LineWidth",2,"Color","r");
        %legend([hReference,hSimulated],"Reference","Simulated", "Location","best")
        %hold off
        view([-31 63]);
    end
end

%% 
% 

function [pos,w,l,h]=randomColumn()
%randomColumn create a random building colum of a given width, length and
%height.
pos=randi([0 200],3,1);
pos(3,1)=0;
w=randi([1,50],1);
l=randi([1,50],1);
h=randi([1,20],1);
end