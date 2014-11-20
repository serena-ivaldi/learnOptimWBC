function [p ,pd ,pdd, sample] = RecordTrajectory(number_of_pivot,step,tf)

   
   figure(1);
   axis([0  tf  0  1])
   for i=1:number_of_pivot
      h(i) = impoint();
      pos = wait(h(i));
   end
   % get the position (x,y coordinates) of each point of the curve
   for i=1:number_of_pivot
      pos = getPosition(h(i));
      val(i) = pos(1);
      sam(i) = pos(2);
   end
   
   sample=0:step:tf;

   % interpolate the points to obtain a continuos curve
   p = interp1(val,sam,sample,'spline'); 
   % saturation to keep value between zero and one
   p(p>=1) = 1;
   p(p<=0) = 0;
   
   % plot positions
   figure;
   plot(sample,p,'.');

   % calculate velocities
   pd = p(2:end) - p(1:end-1);


   % calculate accelerations
   pdd = (pd(2:end) - pd(1:end-1));


   % cut down parts where we
   % don't have acceleration
   pd = pd(2:end);
   p  = p(3:end);
   sample = sample(3:end);

   % % plot x coordinates, velocities and accelerations with respect to x
   % figure;
   % hold on;
   % 
   % plot(x, 'b');
   % plot(dx, 'm'); 
   % plot(ddx, 'r');
   % 
   % legend('x', 'dx', 'ddx');

   % % plot y coordinates, velocities and accelerations with respect to y
   % figure;
   % 
   % hold on;
   % 
   % plot(y, 'b');
   % plot(dy, 'm'); 
   % plot(ddy, 'r');
   % 
   % legend('y', 'dy', 'ddy');
   
   k = waitforbuttonpress(); 
   close all



end
