function add_badge(fig, status, lines, corner)
% add_badge(fig,'pass'|'warn'|'fail', string or cellstr lines, 'nw'|'ne'|'sw'|'se')
if nargin<4, corner='nw'; end
if ischar(lines) || isstring(lines), lines = cellstr(splitlines(string(lines))); end
switch lower(status)
    case 'pass', bg=[0.85 1.00 0.85]; ed=[0.15 0.55 0.15]; ttl='PASS';
    case 'warn', bg=[1.00 0.97 0.85]; ed=[0.70 0.55 0.15]; ttl='WARN';
    otherwise,   bg=[1.00 0.85 0.85]; ed=[0.60 0.15 0.15]; ttl='FAIL';
end
switch lower(corner)
    case 'nw', pos=[0.02 0.78 0.28 0.18];
    case 'ne', pos=[0.70 0.78 0.28 0.18];
    case 'sw', pos=[0.02 0.02 0.28 0.18];
    case 'se', pos=[0.70 0.02 0.28 0.18];
    otherwise,  pos=[0.02 0.78 0.28 0.18];
end
figure(fig); 
txt = [{['\bf' ttl '\rm']} ; lines(:)];
annotation(fig,'textbox',pos,'String',txt,'Units','normalized', ...
    'BackgroundColor',bg,'Color',ed,'EdgeColor',ed,'LineWidth',1.2, ...
    'Margin',8,'FontName','Helvetica','FontSize',10);
end
