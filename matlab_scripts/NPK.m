
n=0:5:300
p=0:5:300
k=0:5:300

figure(1)
set(gcf,'position',[0 0 1920 1080]);
set(gcf,'Name','NP');
[N,P]=meshgrid(n,p)
NP=423.6646+2.2008.*N+1.8742.*P-0.0111.*N.^2-0.0112.*P.^2+0.0076.*N.*P
surf(N,P,NP,'FaceColor','none','EdgeColor',[1 0 0]),
xlabel('氮肥（kg/hm^2）','fontsize',30,'FontWeight','bold'),
ylabel('磷肥（kg/hm^2）','fontsize',30,'FontWeight','bold'),
zlabel('产量（kg/hm^2）','fontsize',30,'FontWeight','bold')
print(gcf,'-dpng','NP.png') 
hold on

figure(2)
set(gcf,'position',[0 0 1920 1080]);
set(gcf,'Name','NK');
[N,K]=meshgrid(n,k)
NK=424.9943+2.5399.*N+1.3300.*K-0.0107.*N.^2-0.0063.*K.^2+0.0034.*N.*K 					
surf(N,K,NK,'FaceColor','none','EdgeColor',[0 1 0]),
xlabel('氮肥（kg/hm^2）','fontsize',30,'FontWeight','bold'),
ylabel('钾肥（kg/hm^2）','fontsize',30,'FontWeight','bold'),
zlabel('产量（kg/hm^2）','fontsize',30,'FontWeight','bold')
print(gcf,'-dpng','NK.png')
hold on

figure(3)
set(gcf,'position',[0 0 1920 1080]);
set(gcf,'Name','PK');
[P,K]=meshgrid(p,k)
PK=419.5029+2.5227.*P+1.5296.*K-0.0111.*P.^2-0.0068.*K.^2+0.0038.*P.*K 
surf(P,K,PK,'FaceColor','none','EdgeColor',[0 0 1]),
xlabel('磷肥（kg/hm^2）','fontsize',30,'FontWeight','bold'),
ylabel('钾肥kg/hm^2）','fontsize',30,'FontWeight','bold'),
zlabel('产量（kg/hm^2）','fontsize',30,'FontWeight','bold')
print(gcf,'-dpng','PK.png')
hold on