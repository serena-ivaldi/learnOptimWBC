function SaveFigures(bot,q,step)

   for i = 1:step:size(q,1)

      q_cur = q(i,:);
      bot.plot(q_cur);
      a = input('continue? ','s');


   end

end