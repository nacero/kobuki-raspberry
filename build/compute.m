% read the transformations file to one big matrix
file = dlmread("odometry6.txt");


tickToMeter = 0.000085292090497737556558; % [m/tick]

x = 0;  % [m]
y = 0;  % [m]

theta = 0; % [rad]
initialtheta = 0;
b = 0.23; % wheelbase d [m]

XX=[]
YY=[]
THETATHETA=[]

for row_index = 1:1:size(file,1) 
	
   if row_index == 1
      initialtheta = theta;  
   end  
  
  
	timestamp = file(row_index, 1);
	
	dLeft = file(row_index, 2);
	dRight = file(row_index, 3);

  mLeft = dLeft * tickToMeter;
  mRight = dRight * tickToMeter;

	if mLeft == mRight
		y = y + sin(theta)*mRight;
    x = x + cos(theta)*mRight;
  else
        
		x = x + (b*(mRight+mLeft))/(2*(mRight-mLeft)) * (sin((mRight-mLeft)/b + theta) - sin(theta));
  	y = y + (b*(mRight+mLeft))/(2*(mRight-mLeft)) * (cos((mRight-mLeft)/b + theta) - cos(theta));

		theta = (mRight-mLeft)/b + theta;

		XX(end+1) = x;
    YY(end +1) = y;
    THETATHETA(end+1) = theta;


    
  end

end


	% Temp = T(row_index - 3 :row_index,1:4);
	% TranslationVector = Temp(1:4,4);
	% Point = TranslationVector; % B+
	
	%Point = Temp*B;

	%if abs(TranslationVector(3) - B(3)) > 5000
	%	"Problematic row is:"
	%	row_index
	%	break;
	%end
	% row_index
	% B = TranslationVector;

	% X(end+1)= Point(1);
	% Y(end+1)=Point(2);
	% Z(end+1)=Point(3);
	%B=Point;


%hold off;
%plot(YY,"r")
%print -djpg X.jpg
%hold on



plot(THETATHETA,"g")
% print -djpg Y.jpg

%figure 2

%hold on
figure 2

axis equal
plot(XX,YY,"b")
% print -djpg Z.jpg

% hold on;
% plot(Y,"g.-")
% plot(Z,"b.-")
% plot(X,"r.-")
% print -djpg combined.jpg
% hold off;


% plot3(X,Y,Z)
% xlabel("X")
% ylabel("Y")
% zlabel("Z")

% title ('something here');
% axis ('on');
% axis image;
% hold on;
% plot3(X,Y,Z,".")

% print -djpg trajectory.jpg


% plot3(X,Y,Z)
% print -djpg trajectory.jpg
