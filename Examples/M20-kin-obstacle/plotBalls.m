function h = plotBalls(c,r,color)
[x,y,z]=sphere;
for j=1:length(r)
    xx=x*r(j)+c(j,1);
    yy=y*r(j)+c(j,2);
    zz=z*r(j)+c(j,3);
    h(j)=surf(xx,yy,zz,...
        'edgecolor','none',...
        'Clipping','off',...
        'facecolor',color);
end

