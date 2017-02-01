%% in order to maximize with a minimization a have to put a minus in front
%% of all the surrogates that are natively built as a maximization problem
function ret = Surrogate(self, x, kind, kappa, xi,varargin)
        if strcmp(kind,'ucb')
            ret = - self.ucb(x, kappa);
        end
        if strcmp(kind,'ei')
            ret = - self.ei(x, xi);
        end
        if strcmp(kind,'poi')
            ret = - self.poi(x, xi);
        end
        if strcmp(kind,'eci')
            ret = - self.eci(x, xi);
        end
        if strcmp(kind,'ecv')
            ret = - self.ecv(x);
        end
        if strcmp(kind,'custom');
            ret = - varargin{1}(x,xi);
        end
end






    