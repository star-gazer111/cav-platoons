function mode = hybridAutomaton(mode, measures, params)
% mode: 'platoon' or 'cruise'
% measures: struct with fields ttc_pre (s), crpf_mag, inVO (bool), headway_ok (bool), speed_ok (bool)
% params: struct with fields TTC_WARN (2.2), CRPF_TH, VO_STRICT (bool)

if strcmp(mode,'platoon')
    if (measures.ttc_pre < params.TTC_WARN) || measures.inVO || (measures.crpf_mag > params.CRPF_TH)
        mode = 'cruise'; % split
    end
else
    if measures.headway_ok && measures.speed_ok
        mode = 'platoon'; % merge
    end
end
end
