% -------------------------------------------------------------------------- %
% OpenSim 4.1: addBucketToArm26.m                                          %
% -------------------------------------------------------------------------- %
% Copyright (c) 2019 Stanford University and the Authors                     %
%                                                                            %
% Author(s): Ross Wilkinson                                                  %
%                                                                            %
% Licensed under the Apache License, Version 2.0 (the "License"); you may    %
% not use this file except in compliance with the License. You may obtain a  %
% copy of the License at http://www.apache.org/licenses/LICENSE-2.0          %
%                                                                            %
% Unless required by applicable law or agreed to in writing, software        %
% distributed under the License is distributed on an "AS IS" BASIS,          %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   %
% See the License for the specific language governing permissions and        %
% limitations under the License.                                             %
% -------------------------------------------------------------------------- %

% Initialize script
% =================
clear; close all; clc;
% Import OpenSim Libraries into Matlab
import org.opensim.modeling.*

% Define the Model file path
% ==========================
[modelName, modelPath] = uigetfile('*.osim');

% Initialize the Model
% ===================================
osimModel = Model( [modelPath modelName] );

% Change Model name
% =================
newModelName = strrep(modelName,'.osim','WithBucket.osim');
osimModel.setName(newModelName);

% Define bucket geometries
% =========================


% Get a reference to the ground object
% ====================================
ground = osimModel.getGround();

% Define the acceleration of gravity
% ==================================
osimModel.setGravity( Vec3(0, -9.80665, 0) );

% Access existing Model bodies and joints
% =======================================
bodySet = osimModel.getBodySet();
jointSet = osimModel.getJointSet();

% Set reference to Model bodies of interest
% =========================================
rightHand = bodySet().get("r_ulna_radius_hand");

% Set reference to Model joints of interest
% =========================================
% groundToPelvis = jointSet().get("ground_pelvis");


% Update rider position
% =====================
% % pelvis_tilt
% pelvis_rx = groundToPelvis.upd_coordinates(0);
% pelvis_rx.set_default_value( deg2rad(-20) );
% % pelvis_tx
% pelvis_tx = groundToPelvis.upd_coordinates(3);
% pelvis_tx.set_default_value(0.67);
% % pelvis_ty
% pelvis_tz = groundToPelvis.upd_coordinates(4);
% pelvis_tz.set_default_value(1.2);
% % pelvis_tz
% pelvis_tz = groundToPelvis.upd_coordinates(5);
% pelvis_tz.set_default_value(0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Construct Bodies and Joints %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Body: Bucket
% ==============
bucket = Body();
bucket.setName('bucket');
bucket.setMass(1.0);
bucket.setMassCenter( Vec3(0,-0.1,0) )
bucket.setInertia( Inertia(0.0024,0.0028,0.0024,0,0,0) );
% Add geometry to the body
bucketMesh = "/Users/rosswilkinson/Documents/opensim-moco-0.4.0/Resources/Code/Matlab/exampleSquatToStand/Geometry/bucket.vtp";
bucketGeometry = Mesh( bucketMesh );
bucketGeometry.setColor( Vec3(1,1,1) );
bucket.attachGeometry(bucketGeometry);
% Add Body to the Model
osimModel.addBody(bucket);

% Joint: Bucket {Pin} Hand 
% ============================
r_handle = PinJoint('r_handle',... % Joint Name
    rightHand,... % Parent Body
    Vec3(0.031,-0.31,0.07),... % Location in Parent Frame
    Vec3(0,0,0),... % Orientation in Parent Frame
    bucket,... % Child Body
    Vec3(0,0,0),... % Location in Child Frame
    Vec3(0,0,0) ); % Orientation in Child Frame
% Rotation
r_handle_rot = r_handle.upd_coordinates(0);
r_handle_rot.setRange([deg2rad(-180), deg2rad(180)]);
r_handle_rot.setName('r_handle_rz');
r_handle_rot.setDefaultValue(deg2rad(0)); % 0 deg. slope
r_handle_rot.setDefaultSpeedValue(0);
r_handle_rot.setDefaultLocked(false)
% Add the PlatformToGround Joint to the Model
osimModel.addJoint(r_handle);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize and Save Model %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Check model for consistency
osimModel.initSystem();
% Save the model to a file
cd(modelPath)
osimModel.print(newModelName);
disp('Arm26 with bucket printed!');
