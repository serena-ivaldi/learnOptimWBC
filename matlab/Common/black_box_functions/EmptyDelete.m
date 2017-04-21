% when we design postprocessing function is necessary to add a fake input
% because without it matlab is gonna consider the function with solely obj as input
% as a method of the class where the obj belongs to.

function EmptyDelete(obj,fake_input)



end