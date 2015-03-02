function PlotTorqueFromDat(controller)
plot(controller.torques{1, 1}');
xlabel('t','FontSize',16);
ylabel(strcat('torque'),'FontSize',16);
h_legend = legend({'J1','J2','J3','J4','J5','J6','J7'});
set(h_legend,'FontSize',15);
set(gca,'Layer','top');
end