%Generating a complex maze by calling function

mazeforfirstthree=randommaze_for_firstthree();  %calling the complex maze function
show(mazeforfirstthree) %displaying the maze in a figure
title("Hopefully, a complex maze","FontSize",18); %title and font size
axis off %removing axis

%%
%Generating a simple maze by calling funtion
mazefortactile=randommaze_for_RRT(); %calling the simple maze function
show(mazefortactile) %displaying the maze in a figure
title("Haha!!! This is less complex","FontSize",18); %title and font size
axis off %removing axis


%%
% Simulation of self designed tactile and Ultrasonic sensor

mapmatrix=mazefortactile.occupancyMatrix; %gets the binary occupancy matrix
maploutinmatrix=transpose(mapmatrix); %transposes the matrix and matrix gets flipped while converting from map to matrix
maps=binaryOccupancyMap(mapmatrix); %recreates the map using the obtained matrix wihout transpose to infalte the size so that it can be implented in cartesian space
show(maps) %opens the map(maze)
currentpos=round(ginput(1)); %user input of start positon by clicking the point on the maze using the left mouse button
goal=round(ginput(1));          %user input of goal positon by clicking the point on the maze using the left mouse button
xtemp=[currentpos(1,1)]; %assigns xtemp
ytemp=[currentpos(1,2)]; %assigns ytemp
lastpsotion=currentpos; %inital value setting
while(1>0) %creating an infinite loop 
    num=randi([1 4],1,1); %calling a random number from 1-4
    if num==1 %if statement
        [currentpos,x,y]=leftUS(maploutinmatrix,currentpos,xtemp,ytemp);     %assinged to 1 and calls the leftUS function
    elseif num==2 %if statement
        [currentpos,x,y]=rightUS(maploutinmatrix,currentpos,xtemp,ytemp); %assinged to 2 and calls the rightUS function
    elseif num==3 %if statement
        [currentpos,x,y]=upUS(maploutinmatrix,currentpos,xtemp,ytemp);      %assinged to 3 and calls the uptUS function
    elseif num==4 %if statement
        [currentpos,x,y]=downUS(maploutinmatrix,currentpos,xtemp,ytemp); %assinged to 4 and calls the downUS function
    end %end statement
    xtemp=[xtemp;x(end,:)]; %updates xtemp
    ytemp=[ytemp;y(end,:)]; %updates ytemp
    show(maps) %opens the maps(maze)
    axis off %turns off axis
    hold on %hold on
    plot(xtemp(end),ytemp(end),Marker="*",MarkerSize=10) %plots the current position of robot as a *
    plot(xtemp,ytemp,Marker=".") %plots the past location of robot as .
    delete(findobj(gca, 'Marker', '.')) %destroys all . markers so that only current positon along with line is destroyed
    pause(0.1) %adds a delay of 0.1 secs 
    num=randi([1 4],1,1); %calling a random number from 1-4
    if num==1 %if statement
        [currentpos,x,y]=left(maploutinmatrix,currentpos,xtemp,ytemp); %assinged to 1 and calls the left function
    elseif num==2 %if statement
        [currentpos,x,y]=right(maploutinmatrix,currentpos,xtemp,ytemp); %assinged to 2 and calls the right function
    elseif num==3 %if statement
        [currentpos,x,y]=up(maploutinmatrix,currentpos,xtemp,ytemp); %assinged to 3 and calls the up function
    elseif num==4 %if statement
        [currentpos,x,y]=down(maploutinmatrix,currentpos,xtemp,ytemp); %assinged to 4 and calls the down function
    end %end statement
    xtemp=[xtemp;x(end,:)]; %updates xtemp
    ytemp=[ytemp;y(end,:)]; %updates ytemp
    show(maps) %opens the maps(maze)
    axis off %turns off axis
    hold on %hold on
    plot(xtemp(end),ytemp(end),Marker="*",MarkerSize=10) %plots the current position of robot as a *
    plot(xtemp,ytemp,Marker=".") %plots the past location of robot as .
    delete(findobj(gca, 'Marker', '.')) %destroys all . markers so that only current positon along with line is destroyed
    pause(0.1) %adds a delay of 0.1 secs 
    if xtemp(end)==goal(1,1) && ytemp(end)==goal(1,2) %checks if goal is reached 
        break; %break statement
    end %end statement
end %end statement
hold off %turning off hold


%%
%Finding start and goal points for complex maze if user doesnt want to
%click points on the maze by interative point clicker

binarymatrix=mazeforfirstthree.occupancyMatrix; %gets the binary matrix for the complex maze
[m, n]=size(binarymatrix); %gets the size of the matrix using m and n as outputs
for ii=m:-1:1 %reverse for loop for m to 1 for start location
    for jj=n:-1:1 %reverse nested for loop for n to 1
        if binarymatrix(ii,jj)==0 %if statement to check whether a certain point it is 0
            start=[ii jj]; %assigns start location 
            break %break statement for innner loop
        end %end statement
    end %end statement
    if binarymatrix(ii,jj)==0 %if statement to check whether a certain point it is 0
        break %break statement for outer loop
    end %end statement
end %end statement

for ii=1:m % for loop for m to 1 for end location
    for jj=1:n % for loop for m to 1 for end location
        if binarymatrix(ii,jj)==0 %if statement to check whether a certain point it is 0
            goal=[ii jj]; %assigns goal location 
            break %break statement for innner loop
        end %end statement
    end %end statement
    if binarymatrix(ii,jj)==0 %if statement to check whether a certain point it is 0
        break %break statement for outer loop
    end %end statement
end %end statement

%%
%Bug 2 algorithm implementation in a complex maze

show(binaryOccupancyMap(mazeforfirstthree.getOccupancy)) %opens the maze for the bug2 algorithim
Tactilesensor=Bug2(mazeforfirstthree.occupancyMatrix); %adds algorithm to the maze
Tactilesensor.query(round(ginput(1)),round(ginput(1)),'animate') %given start and goal locations by user input, animates the maze
title("Bug 2 Algorithm Path (Not Fun))","FontSize",18); %adds title
axis off %turns off axis 

%%
% D star implementation for complex maze

show(binaryOccupancyMap(mazeforfirstthree.getOccupancy)) %opens the maze for D*
goal=round(ginput(1)) %takes user input for goal by left mouse click
costoptimisation=Dstar(mazeforfirstthree.occupancyMatrix); %creates a costmap by adding dstar function to the maze
c=costoptimisation.costmap(goal); %optimises the cost by giving the goal
costoptimisation.plan(goal); %plans the cost by giving goal
costoptimisation.niter; %displays the interation number
costoptimisation.query(round(ginput(1)),'animate') %displays path given input for start location by user click on maze
title("Yep, this is fast!","FontSize",18); %title heading
axis off %axis off

%%
%A star implementation for complex maze
rng('default'); %random number generator
map = mapClutter; %using map clutter command creates a map
planner = plannerAStarGrid(mazeforfirstthree); %creates a star grid using input from the complex maze
show(planner) %shows the planner
plan(planner,round(ginput(1)),round(ginput(1))); %plans path using input for start and goal using user click on maze
show(planner) %opens planner agian
title("If only we could make it better...","FontSize",18); %adds title 
axis off %turns off axis

%%
% RRT for less complex maze
ss = stateSpaceSE2; %creates a state space object
sv = validatorOccupancyMap(ss); %makes a validator occupancy map with the state space
sv.Map=mazefortactile; %adds the simple maze to the sv object
sv.ValidationDistance = 0.2; %adds the validation distance set by trail and error method
ss.StateBounds = [mazefortactile.XWorldLimits;mazefortactile.YWorldLimits; [-pi pi]]; %creates the state bounds by assigning them as the world limits and [-180 180] range
planner = plannerRRT(ss,sv);         %creates a RRT planner
planner.MaxConnectionDistance =0.5; %creates the max connection distance for the planner
show(mazefortactile) %opens the simple maze
rng(70,'twister'); % for repeatable result
[pthObj,solnInfo] = plan(planner,[round(ginput(1)) 0],[round(ginput(1)) 0]); %outputs path and solution by planning path for given inputs by user clicks on map for start and goal and no roation 
hold on %turns on hold
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-');       % tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2) % draw path
hold off %turns off hold
title("Fancy trees for a simple maze","FontSize",18); %adds title
axis off %axis off

%%
% PRM for complex/simple maze


prmComplex = mobileRobotPRM(mazefortactile,500); %creates a PRM for simple maze with 500 nodes
show(mazefortactile) %shows the maze
show(prmComplex); %shows the prmComplex
path = findpath(prmComplex, round(ginput(1)), round(ginput(1))); %finds the path by taking user input by mouse click for start and goal locations 
show(prmComplex); %opens the PRM complex object
title("Lame, could be more optimal.","FontSize",18); %creates a title 
axis off %turns off axis

%%
% All functions

%---------------------------------------------------------------------------
%function for complex maze for D*/A*/Bug2 algorithim
%inbuilt matlab function which creates a binary occupancy matrix for a maze by giving parmeters
%parmeters given here was only the map size and only 10X10 so that,
%complexity and computation power is balaced.
%---------------------------------------------------------------------------
function generated_maze_for_firstthree=randommaze_for_firstthree() %output is generated_maze_for_firstthree and no input needed
    generated_maze_for_firstthree = mapMaze('MapSize',[10 10]);      %mapMaze() is the inbuilt function
end %function end statement
%---------------------------------------------------------------------------






%---------------------------------------------------------------------------
%function for simple maze for RRT/PRM/custom tactile and Ultrasonic sensors
%inbuilt matlab function which creates a binary occupancy matrix for a maze by giving parmeters
%parmeters given here was only the map size and only 10X10 and map resoultion was set to 3,
%complexity is lowered and computation power is increased for complex and time consuming alogorithims.
%---------------------------------------------------------------------------
function generated_maze_for_RRT=randommaze_for_RRT() %output is generated_maze_for_firstthree and no input needed
    generated_maze_for_RRT = mapMaze('MapSize',[10 10],'MapResolution',3); %mapMaze() is the inbuilt function
end %function end statement
%---------------------------------------------------------------------------







%---------------------------------------------------------------------------
%function for simple maze for left side movement using Ultrasonic sensor
%Compares the binary occupancy matirx(maze) so if a 5 grid movement is-
%possible it will execute it, else it will reach a grid right before it is
%occupied.
%---------------------------------------------------------------------------
function [currentpos,xtemp,ytemp]=leftUS(maploutinmatrix,currentpos,xtemp,ytemp) %function outputs are currentpos,xtemp,ytemp and inputs are the binarymatrix,currentpos,xtemp,ytemp
    for i=1:5 %initalises for loop running 5 iterations
        if maploutinmatrix(currentpos(1,1)-1,30-currentpos(1,2)+1)==0 %checks if left position is occupied or not
            lastposition=[currentpos(1,1) currentpos(1,2)]; %takes the last position in consideration
            xtemp=[xtemp;currentpos(1,1)-1]; %xtemp gets updated for each iteration
            ytemp=[ytemp;currentpos(1,2)]; %ytemp gets updated for each iteration
            currentpos=[currentpos(1,1)-1 currentpos(1,2)]; %current position gets updated for each iteration
        end %end statement
    end %end statement
end %end statement
%---------------------------------------------------------------------------






%---------------------------------------------------------------------------
%function for simple maze for Right side movement using Ultrasonic sensor
%Compares the binary occupancy matirx(maze) so if a 5 grid movement is-
%possible it will execute it, else it will reach a grid right before it is
%occupied.
%---------------------------------------------------------------------------
function [currentpos,xtemp,ytemp]=rightUS(maploutinmatrix,currentpos,xtemp,ytemp) %function outputs are currentpos,xtemp,ytemp and inputs are the binarymatrix,currentpos,xtemp,ytemp
    for i=1:5 %initalises for loop running 5 iterations
        if maploutinmatrix(currentpos(1,1)+2,30-currentpos(1,2)+1)==0 %checks if right position is occupied or not
            lastposition=[currentpos(1,1) currentpos(1,2)]; %takes the last position into consideration
            xtemp=[xtemp;currentpos(1,1)+1]; %xtemp gets updated for each iteration
            ytemp=[ytemp;currentpos(1,2)]; %ytemp gets updated for each iteration
            currentpos=[currentpos(1,1)+1 currentpos(1,2)]; %current position gets updated for each iteration
        end %end statement
    end %end statement
end %end statement
%---------------------------------------------------------------------------






%---------------------------------------------------------------------------
%function for simple maze for Upward movement using Ultrasonic sensor
%Compares the binary occupancy matirx(maze) so if a 5 grid movement is-
%possible it will execute it, else it will reach a grid right before it is
%occupied.
%---------------------------------------------------------------------------
function [currentpos,xtemp,ytemp]=upUS(maploutinmatrix,currentpos,xtemp,ytemp) %function outputs are currentpos,xtemp,ytemp and inputs are the binarymatrix,currentpos,xtemp,ytemp
    for i=1:5 %initalises for loop running 5 iterations
        if maploutinmatrix(currentpos(1,1),30-currentpos(1,2)-1)==0 %checks if up position is occupied or not
            lastposition=[currentpos(1,1) currentpos(1,2)]; %takes the last position into consideration
            xtemp=[xtemp;currentpos(1,1)]; %xtemp gets updated for each iteration
            ytemp=[ytemp;currentpos(1,2)+1]; %ytemp gets updated for each iteration
            currentpos=[currentpos(1,1) currentpos(1,2)+1]; %current position gets updated for each iteration
        end %end statement
    end %end statement
end %end statement
%---------------------------------------------------------------------------






%---------------------------------------------------------------------------
%function for simple maze for downward movement using Ultrasonic sensor
%Compares the binary occupancy matirx(maze) so if a 5 grid movement is-
%possible it will execute it, else it will reach a grid right before it is
%occupied.
%---------------------------------------------------------------------------
function [currentpos,xtemp,ytemp]=downUS(maploutinmatrix,currentpos,xtemp,ytemp) %function outputs are currentpos,xtemp,ytemp and inputs are the binarymatrix,currentpos,xtemp,ytemp
    for i=1:5 %initalises for loop running 5 iterations
        if maploutinmatrix(currentpos(1,1),30-currentpos(1,2)+2)==0 %checks if down position is occupied or not
            lastposition=[currentpos(1,1) currentpos(1,2)]; %takes the last position into consideration
            xtemp=[xtemp;currentpos(1,1)]; %xtemp gets updated for each iteration
            ytemp=[ytemp;currentpos(1,2)-1]; %ytemp gets updated for each iteration
            currentpos=[currentpos(1,1) currentpos(1,2)-1]; %current position gets updated for each iteration
        end %end statement
    end %end statement
end %end statement
%---------------------------------------------------------------------------





%---------------------------------------------------------------------------
%function for simple maze for left side movement using tactile sensor
%Compares the binary occupancy matirx(maze) so if a 1 grid movement is-
%possible it will execute it, else it will not execute.
%---------------------------------------------------------------------------
function [currentpos,xtemp,ytemp]=left(maploutinmatrix,currentpos,xtemp,ytemp) %function outputs are currentpos,xtemp,ytemp and inputs are the binarymatrix,currentpos,xtemp,ytemp
    if maploutinmatrix(currentpos(1,1)-1,30-currentpos(1,2)+1)==0 %checks if left position is occupied or not
        lastposition=[currentpos(1,1) currentpos(1,2)]; %takes the last position into consideration
        xtemp=[xtemp;currentpos(1,1)-1]; %xtemp gets updated
        ytemp=[ytemp;currentpos(1,2)]; %ytemp gets updated
        currentpos=[currentpos(1,1)-1 currentpos(1,2)]; %current position gets updated
    end %end statement
end %end statement
%---------------------------------------------------------------------------





%---------------------------------------------------------------------------
%function for simple maze for right side movement using tactile sensor
%Compares the binary occupancy matirx(maze) so if a 1 grid movement is-
%possible it will execute it, else it will not execute.
%---------------------------------------------------------------------------
function [currentpos,xtemp,ytemp]=right(maploutinmatrix,currentpos,xtemp,ytemp) %function outputs are currentpos,xtemp,ytemp and inputs are the binarymatrix,currentpos,xtemp,ytemp
    if maploutinmatrix(currentpos(1,1)+2,30-currentpos(1,2)+1)==0 %checks if right position is occupied or not
        lastposition=[currentpos(1,1) currentpos(1,2)]; %takes the last position into consideration
        xtemp=[xtemp;currentpos(1,1)+1]; %xtemp gets updated 
        ytemp=[ytemp;currentpos(1,2)]; %ytemp gets updated 
        currentpos=[currentpos(1,1)+1 currentpos(1,2)]; %current position gets updated
    end %end statement
end %end statement
%---------------------------------------------------------------------------






%---------------------------------------------------------------------------
%function for simple maze for upward movement using tactile sensor
%Compares the binary occupancy matirx(maze) so if a 1 grid movement is-
%possible it will execute it, else it will not execute.
%---------------------------------------------------------------------------
function [currentpos,xtemp,ytemp]=up(maploutinmatrix,currentpos,xtemp,ytemp) %function outputs are currentpos,xtemp,ytemp and inputs are the binarymatrix,currentpos,xtemp,ytemp
    if maploutinmatrix(currentpos(1,1),30-currentpos(1,2)-1)==0 %checks if up position is occupied or not
        lastposition=[currentpos(1,1) currentpos(1,2)]; %takes the last position into consideration
        xtemp=[xtemp;currentpos(1,1)]; %xtemp gets updated 
        ytemp=[ytemp;currentpos(1,2)+1]; %ytemp gets updated 
        currentpos=[currentpos(1,1) currentpos(1,2)+1]; %current position gets updated
    end %end statement
end %end statement
%---------------------------------------------------------------------------






%---------------------------------------------------------------------------
%function for simple maze for downward movement using tactile sensor
%Compares the binary occupancy matirx(maze) so if a 1 grid movement is-
%possible it will execute it, else it will not execute.
%---------------------------------------------------------------------------
function [currentpos,xtemp,ytemp]=down(maploutinmatrix,currentpos,xtemp,ytemp) %function outputs are currentpos,xtemp,ytemp and inputs are the binarymatrix,currentpos,xtemp,ytemp
    if maploutinmatrix(currentpos(1,1),30-currentpos(1,2)+2)==0 %checks if down position is occupied or not
        lastposition=[currentpos(1,1) currentpos(1,2)]; %takes the last position into consideration
        xtemp=[xtemp;currentpos(1,1)]; %xtemp gets updated
        ytemp=[ytemp;currentpos(1,2)-1]; %ytemp gets updated
        currentpos=[currentpos(1,1) currentpos(1,2)-1]; %current position gets updated
    end %end statement
end %end statement
%---------------------------------------------------------------------------