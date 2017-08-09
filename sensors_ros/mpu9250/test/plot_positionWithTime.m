dt=[dt];
position=[position];
plot(dt,position);

for i=1:5
    y2=polyfit(dt,position,i);
    Y=polyval(y2,dt);%计算拟合函数在x处的值。
    if sum((Y-position).^2)<0.1
            c = i
        break;
    end
end

y1=polyfit(dt,position,i)