% In this script, it is evaluated how PGR behave when a position in x and y
% axis changes a little bit of its initial position. The initial grasp is a
% Powergrasp.
clear all; close all; clc;

% With help of Syn_GUI, I saw that between this range of values, the grasp
% is stable, let's see the plot to be sure
load('CubePos.mat');
obj0 = obj;
hand0 = VizzyHandModel;
hand0.S = hand.S;
%PGRtypes = ["BF", "H2", "H3", "H4"]; % PGR different types to be computed
PGRtypes = ["BF"];
x = 65:3:85;
y = -25:4:4;

[X,Y] = meshgrid(x,y);
n_rows = size(X,1);
n_col = size(X,2);
PGR = zeros(n_rows, n_col);
PGR_for_variance = cell(n_rows,n_col);
z_star=1.96;
sigma_=10;
MOE = 5;
number_of_samples = ceil((z_star*sigma_/MOE)^2);
delta_x = 3;
delta_y= 4;
delta_ang = 2*1.6*pi/180;
%%
for i = 1: n_rows
    waitbar(i/n_rows);
    
    for j = 1: n_col
        % Reset hand and obj
        
        rand_x = randn(1,number_of_samples);
        my_x_values = x(i)+delta_x*rand_x/3;
        rand_y = randn(1,number_of_samples);
        my_y_values = y(i)+delta_y*rand_y/3;
        rand_rot_x = randn(1,number_of_samples);
        my_rot_x_values = delta_ang*rand_rot_x/3;
        rand_rot_y = randn(1,number_of_samples);
        my_rot_y_values = delta_ang*rand_rot_y/3;
        rand_rot_z = randn(1,number_of_samples);
        my_rot_z_values = delta_ang*rand_rot_z/3;
        for x_pos=my_x_values
            for y_pos=my_y_values
                for rot_x=my_rot_x_values
                    for rot_y=my_rot_y_values
                        for rot_z=my_rot_z_values
                
                            hand =  hand0;
                            obj = obj0;
                            obj.center(1) = x_pos;
                            obj.center(2) = y_pos;
                            rot = zeros(3,1);
                            rot(1)=rot_x;
                            rot(2)=rot_y;
                            rot(3)=rot_z;
                            obj = SGrebuildObject(obj, obj.center, rot); % Rebuilds a new object

                            % Close the hand
                            [hand, obj] = SGcloseHandWithSynergiesV2(hand,obj);

                            % Quality metrics
                            ind = (i-1)*n_col +j;
                            Title = ['Pos: (x,y)= (',num2str(X(i,j)),',' num2str(Y(i,j)),')'];
                            tic
                            PGR_ = ComputePGRWithBFandHeur(hand, obj, 'BF');
                            %Plot1Hand_Object(hand, obj, Title, PGR,ind);
                            toc
                            %PGR(i,j) = PGR_.BF;
                            curr_vector = PGR_for_variance{i,j};
                            curr_vector = [curr_vector PGR_.BF];
                            PGR_for_variance{i,j}=curr_vector;
                        end
                    end
                end
            end
        end
    end
    close all;
end
figure();
surf(X,Y,PGR);


%% Variation in the X axis

% How much mm can an object move and the grasp is still be considered stable

% Pre allocation of memory
for type = PGRtypes
    xPosLen.(type) = zeros(n_rows, n_col);
end


for i = 1: n_rows
    waitbar(i/n_rows);
    
    for j = 1: n_col
        % Reset hand and obj
        hand =  hand0;
        obj = obj0;
        obj.center(1) = X(i,j);
        obj.center(2) = Y(i,j);
        rot = zeros(3,1);
        obj = SGrebuildObject(obj, obj.center, rot); % Rebuilds a new object

        axisGrasps = ComputePosVariance41Axis(obj, 'x', 5, 5, PGRtypes);
        for type = PGRtypes
            xPosLen.(type)(i,j) = axisGrasps.PosLen.(type);
        end
    end
    close all;
end

%%
for type = PGRtypes
    figure();
    surf(X,Y,xPosLen.(type));
    xlabel('X')
    ylabel('Y');
    zlabel('Maxvar in Xaxis')
    titl = sprintf('(X,Y)-> obj center Pos\n(Z)-> Max X Var a grasp can resist\n PGR=%s',type);
    title(titl);
end

%% Variation in the Y axis

% How much mm can an object move and the grasp is still be considered stable

% Pre allocation of memory
for type = PGRtypes
    yPosLen.(type) = zeros(n_rows, n_col);
end



for i = 1: n_rows
    waitbar(i/n_rows);
    
    for j = 1: n_col
        % Reset hand and obj
        hand =  hand0;
        obj = obj0;
        obj.center(1) = X(i,j);
        obj.center(2) = Y(i,j);
        rot = zeros(3,1);
        obj = SGrebuildObject(obj, obj.center, rot); % Rebuilds a new object

        axisGrasps = ComputePosVariance41Axis(obj, 'y', 5, 5, PGRtypes);
        for type = PGRtypes
            yPosLen.(type)(i,j) = axisGrasps.PosLen.(type);
        end
    end
    close all;
end

%%
for type = PGRtypes
    figure();
    surf(X,Y,yPosLen.(type));
    xlabel('X')
    ylabel('Y');
    zlabel('Maxvar in Yaxis')
    titl = sprintf('(X,Y)-> obj center Pos\n(Z)-> Max Y Var a grasp can resist\n PGR=%s',type);
    title(titl);
end

%% Save workspace
save RobPosXY.mat

%% Computing the square error between Variance of X and Y axis computed with PGR brute force and the other heuristics


PGRHeurTypes = ["H2", "H3", "H4"]; % Name of PGR Heuristics computed

T = [];
for type = PGRHeurTypes
    % X axis
    XErrorMat = (xPosLen.BF - xPosLen.(type)).^2;
    XVarError.(type) = sum(XErrorMat, 'all');
    
    % Y axis
    YErrorMat = (yPosLen.BF - yPosLen.(type)).^2;
    YVarError.(type) = sum(YErrorMat, 'all');
    T2 = table([XVarError.(type);YVarError.(type)]);
    T2.Properties.VariableNames = {char(type)};
    T = [T, T2];
end

T.Properties.RowNames{1} = 'X axis Error';
T.Properties.RowNames{2} = 'Y axis Error';
disp('Sum of error.^2 between Var Pos Mat with BF and the other heuristics:');
disp(T);
