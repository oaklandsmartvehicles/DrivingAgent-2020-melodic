close all
t= 0:1/5000:5;                   % 10 seconds @ 5kHz sample rate
fo=10;f1=300;                    % Start at 10Hz, go up to 400Hz
y=(chirp(t,fo,2,f1,'logarithmic')>0)*1024;
figure;plot(t,y);
y=y+100*rand(size(y));
figure
spectrogram(y,256,200,256,1000);
figure
c_micros1=[0,0,0];
value=[0,0,0];
speeddata=[];
ii=1;
i = 0;
while(1)
  c_val = 0;
  p_val = 0;
  count = 0;
  udcount = 0;
  ud = 0;
  duration1 = 0.0;
  n_micros = t(ii)*1000000 + 50000;
  ii = ii + 1;
  while(1)
    c_micros = t(ii)*1000000;
    value(i+1) = y(ii);
    c_micros1(i+1)=c_micros;
    ii = ii + 1;
    i = i + 1;
    if(i > 2) i = 0;end
    c_time = (c_micros1(1) + c_micros1(2) + c_micros1(3)) / 3.0;
    if((value(1) >= value(2)) && (value(2) > value(3))) c_val = value(1);end
    if((value(1) >= value(3)) && (value(3) > value(2))) c_val = value(3);end
    if((value(2) >= value(1)) && (value(1) > value(3))) c_val = value(1);end
    if((value(2) >= value(3)) && (value(3) > value(1))) c_val = value(3);end
    if((value(3) >= value(1)) && (value(1) > value(2))) c_val = value(1);end
    if((value(3) >= value(2)) && (value(2) > value(1))) c_val = value(2);end
    c_val = value(1);
    if(c_val - p_val > 500) ud_duration(ud+1) = c_time;ud = ud + 1;count = count +1;end
    if(p_val - c_val > 500) ud_duration(ud+1) = c_time;ud = ud + 1;count = count +1;end
    if(ud > 1) 
      duration1 = duration1 + (ud_duration(2)-ud_duration(1));
      ud_duration(1)=ud_duration(2);
      ud = 1;udcount = udcount + 1;
    end
    p_val = c_val;
    if(n_micros < c_micros) break;end
  end
  total = c_micros - n_micros + 50000;
  fcount = count;
  if(udcount > 0) fcount = udcount +  (total - duration1) * udcount / duration1;end
  speeddata=[speeddata,(total - duration1) * udcount / duration1];
end
plot(speeddata)


