function [status, msg] = assess_exp1(R, delay_s, tolTTC, tolTET, robustMax)
% Hard checks from paper: for CD ≤ 0.05 s, expect minTTC > 2.0 and TET ≈ 0
if nargin<3, tolTTC=0.05; end
if nargin<4, tolTET=0.20; end
if nargin<5, robustMax=0.05; end
minTTC = min(R.minTTC);  maxTET = max(R.TET);
allPlatoonEnd = all(R.modeEnd=="platoon");

% Advisory: did SV-3 split and later re-merge?
advisory = "n/a";
try
    sv3_mode = R.logs.SV(3).mode; t = R.logs.t;
    iSplit = find(sv3_mode=="cruise",1,'first');
    iMerge = []; if ~isempty(iSplit), iMerge = find((sv3_mode=="platoon") & ((1:numel(t))'>iSplit),1,'first'); end
    if ~isempty(iSplit)
        advisory = sprintf('SV3 split @%.1fs, remerge %s', t(iSplit), tern(~isempty(iMerge),sprintf('@%.1fs',t(iMerge)),'(none)'));
    else
        advisory = 'SV3 split not observed';
    end
catch
    advisory = 'SV3 advisory unavailable';
end

if delay_s <= robustMax
    hardPass = (minTTC > (2.0 - tolTTC)) && (maxTET <= tolTET);
    if hardPass && allPlatoonEnd, status='pass';
    elseif hardPass,               status='warn';
    else,                          status='fail';
    end
else
    % For >0.05 s, degradation is expected — report informationally:
    status = tern((minTTC > (2.0 - tolTTC)) && (maxTET <= tolTET),'warn','fail');
end

msg = sprintf('delay=%.3fs | minTTC=%.2fs | TET=%.2fs\nend platoon=%d\n%s', ...
              delay_s, minTTC, maxTET, allPlatoonEnd, advisory);
end

function [status, msg] = assess_exp2(R2, delay_s, tolTTC, tolTET, robustMax)
if nargin<3, tolTTC=0.05; end
if nargin<4, tolTET=0.30; end
if nargin<5, robustMax=0.05; end
minTTC = min(R2.minTTC);  maxTET = max(R2.TET);  eff = sum(R2.eff_dev);

if delay_s <= robustMax
    status = tern((minTTC > (2.0 - tolTTC)) && (maxTET <= tolTET),'pass','fail');
else
    % Degradation expected at higher delays → warn unless outright unsafe
    status = tern((minTTC > 1.8) && (maxTET <= 2.0),'warn','fail');
end
msg = sprintf('delay=%.3fs | minTTC=%.2fs | TET=%.2fs | ΣEffDev=%.1f', delay_s, minTTC, maxTET, eff);
end

function out = tern(cond, a, b), if cond, out=a; else, out=b; end
end
