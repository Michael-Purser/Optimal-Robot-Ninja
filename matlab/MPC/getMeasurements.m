function MPC = getMeasurements(MPC,veh,env)

fprintf('Getting measurements \n');

MPC = sensor(MPC,veh,env);

end