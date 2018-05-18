if exitFlagQP_unsuccessful.Data(end)
    disp("Early termination due to unsuccessful QP");
elseif no_force_on_feet.Data(end)
    disp("Early termination due to no forces on feet");
elseif ZMP_out_of_bounds.Data(end)
    disp("Early termination due to ZMP out of support polygon");
else
    disp("Normal termination at end of simulation time");
end