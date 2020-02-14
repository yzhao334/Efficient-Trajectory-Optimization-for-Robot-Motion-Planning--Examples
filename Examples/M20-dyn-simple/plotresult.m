% load result;
clf;
dis=30;
viewangle=[150,30];
h = plot_m16_3D_custom( opt.Xopt(:,1:6), prob.si.DH, prob.rob.rtb.tool, prob.parts, viewangle, dis, 1, 1 );
h = plot_repeat(flipud(opt.Xopt(:,1:6)),prob.si.DH,prob.rob.rtb.tool,prob.parts,dis,0,h);
% for k=1:length(h)
%     set(h(k),'facealpha',0.2);
% end
for i=1:5
    h = plot_repeat(opt.Xopt(:,1:6),prob.si.DH,prob.rob.rtb.tool,prob.parts,dis,0,h);
%     for k=1:length(h)
%         set(h(k),'facealpha',0.2);
%     end
    h = plot_repeat(flipud(opt.Xopt(:,1:6)),prob.si.DH,prob.rob.rtb.tool,prob.parts,dis,0,h);
%     for k=1:length(h)
%         set(h(k),'facealpha',0.2);
%     end
end