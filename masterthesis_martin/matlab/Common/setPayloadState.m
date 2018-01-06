function setPayloadState(plstate, tidx, varargin)
    global gbl_plstate;
    if isempty(gbl_plstate)
        error('setPayloadState: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
    end
    if ~islogical(plstate)
        error('setPayloadState: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end

    switch nargin
        case {2, 4}
            % set the global state values ...
            if (~gbl_plstate.obj_grabbed && plstate)
                % object grabbed:
                gbl_plstate.obj_grabbed = plstate;
                if (tidx > 0)
                    gbl_plstate.obj_released = false;
                    gbl_plstate.tidx_go      = tidx;
                end

                if (nargin == 4)
                    % qj   = varargin{1}
                    % fval = varargin{2}
                    gbl_plstate.qj_go   = varargin{1,1};
                    gbl_plstate.fval_go = varargin{1,2};
                end
            elseif (gbl_plstate.obj_grabbed && ~gbl_plstate.obj_released && ~plstate)
                % object released:
                gbl_plstate.obj_grabbed = plstate;
                if (tidx > 0)
                    gbl_plstate.obj_released = true;
                    gbl_plstate.tidx_ro      = tidx;
                end

                if ~isempty(gbl_plstate.qj_go)
                    % reset values ...
                    gbl_plstate.qj_go   = [];
                    gbl_plstate.fval_go = [];
                end
            else
                error('setPayloadState: The payload state is ambiguous!');
            end
        otherwise
            error('setPayloadState: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
end
