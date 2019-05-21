function MPC = sendActuationSignal(MPC,veh,localPlanner)

fprintf('Sending actuation signal to robot \n');

MPC = actuate(MPC,veh,localPlanner,2);