%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2014
%
% Homework 5
%
% Team number: 10
% Team leader: Brian Slakter (bjs2135)
% Team members: Sinjihn Smith (sss2208), Sheng Qian(sq2168)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% run 1 as second argument to run part 1
% run 2 as second argument to run part 2

function hw5_team_10(serPort, hwPart)

    global cam_url;
    cam_url = 'http://192.168.0.101/img/snapshot.cgi?f';

	if hwPart == 1
		hw5_part1(serPort);
	elseif hwPart == 2
		hw5_part2(serPort);
    elseif hwPart == 3
        hw5_part3(serPort);
	end

end

%Color tracking component of homework
function hw5_part1(serPort)

	% take image
    global cam_url;
    
	pic = imread(cam_url);

	% choose pixel for color to identify
	image_figure = figure(1);
	image(pic);
	point_clicked = round(ginput(1));
    	point_clicked_y = point_clicked(2);
    	point_clicked_x = point_clicked(1);
            %disp('registered click');
	close(image_figure);
            %disp('closed');

	% convert RGB to HSV
	pic = rgb2hsv(pic);
    
        %disp('converted');

	% smooth image using provided gaussian filter
	smoothed_image = gaussfilt(pic,3);
    	smoothed_image = decorrstretch(smoothed_image);
            %disp('filtered');

	% extract hue, saturation, and values from pixel chosen and surrounding pixels (11x11 area, pixel chosen in center)
	point_clicked_x_max = point_clicked_x + 2;
	point_clicked_y_max = point_clicked_y + 2;
	point_clicked_x = point_clicked_x - 2;
	point_clicked_y = point_clicked_y - 2;
	image_area_pixel_count = point_clicked_x_max - point_clicked_x;
	color_chosen_hue = 0;
	color_chosen_saturation = 0;
	color_chosen_value = 0;
    
            disp('initialize');

    colors = zeros(image_area_pixel_count, 3);
	i = 1;
	while point_clicked_x <= point_clicked_x_max
		color_chosen_hue = smoothed_image(point_clicked_y, point_clicked_x, 1);
		color_chosen_saturation = smoothed_image(point_clicked_y, point_clicked_x, 2);
		color_chosen_value = smoothed_image(point_clicked_y, point_clicked_x, 3);
        		point_clicked_x = point_clicked_x + 1;
        		colors(i,1) = color_chosen_hue;
        		colors(i,2) = color_chosen_saturation;
        		colors(i,3) = color_chosen_value;
        		i = i+1;
   	 end
    
            %disp('gathered surrouding and set color based on pixels');

	% we now have determined which color we will be tracking - we can start taking live images	
	[initial_pic, initial_centroid, initial_pixel_count] = handle_image(pic, colors);
    %disp(initial_pixel_count);

	while 1

		[pic, centroid, pixel_count] = handle_image(pic, colors);
		centroid_x = centroid(2);
		pic_center = size(pic)/2;
		pic_center_x = pic_center(2);
		lateral_change = (pic_center_x - centroid_x)/pic_center_x;
		pixel_change = (initial_pixel_count - pixel_count) / initial_pixel_count;
        adjust_roomba(serPort, pixel_change,lateral_change);

	end

end


function hw5_part2(serPort)

    %get near center of hall
    Spiral(serPort);
	% take image
     idealcolors = ...
    [0.5618    0.2651    0.4215; ...
     0.5626    0.2660    0.4255; ...
     0.5632    0.2677    0.4297; ...
     0.5640    0.2700    0.4341; ...
     0.5649    0.2722    0.4386];
    global cam_url;

    colors = idealcolors;
    
    pic = imread(cam_url);

	% we now have determined which color we will be tracking - we can start taking live images	
	[initial_pic, initial_centroid, initial_pixel_count] = handle_image(pic, colors);
    disp(initial_pixel_count);
    sizeP=size(pic);
    sizeP = sizeP(1)*sizeP(2); 
    pixel_count=initial_pixel_count;
    
    count = 0;
    
    %find a door large enough
    while pixel_count/sizeP<0.1
        
        disp(pixel_count);
        disp(sizeP);
        Ang=0;
        count=count+1;
        AngleSensorRoomba(serPort);
        while abs(Ang)<10/180*pi
            SetFwdVelAngVelCreate(serPort,0,0.2);
            pause(0.01);
            Ang=Ang+AngleSensorRoomba(serPort);
        end
		[pic, centroid, pixel_count] = handle_image(pic, colors);
		sizeP = size(pic); 
        sizeP = sizeP(1)*sizeP(2); 
        if count >= 36
            Ang1=0;
            AngleSensorRoomba(serPort);
            while abs(Ang1)<pi/2
                SetFwdVelAngVelCreate(serPort,0,0.2);
                pause(0.01);
                Ang1=Ang1+AngleSensorRoomba(serPort);
            end
            SetFwdVelAngVelCreate(serPort,0.25,0);
            pause(2);
            count = 0;
        end
    end
    initial_centroid=centroid;
    sizeP=size(pic);
    sizeP=sizeP(2)/2;
    
    %do adjustment to get better angle to door
    if initial_centroid<sizeP
        flagleft=1;
        Ang=0;
        AngleSensorRoomba(serPort);
        while abs(Ang)<pi/4
            SetFwdVelAngVelCreate(serPort,0,-0.2);
            pause(0.01);
            Ang=Ang+AngleSensorRoomba(serPort);
        end
        SetFwdVelAngVelCreate(serPort,0.25,0);
        pause(2);
        Ang=0;
        AngleSensorRoomba(serPort);
        while abs(Ang)<2*pi/5
            SetFwdVelAngVelCreate(serPort,0,0.2);
            pause(0.01);
            Ang=Ang+AngleSensorRoomba(serPort);
        end
        SetFwdVelAngVelCreate(serPort,0.25,0);
    else
        flagleft=0;
        Ang=0;
        AngleSensorRoomba(serPort);
        while abs(Ang)<pi/4
            SetFwdVelAngVelCreate(serPort,0,0.2);
            pause(0.01);
            Ang=Ang+AngleSensorRoomba(serPort);
        end
        SetFwdVelAngVelCreate(serPort,0.25,0);
        pause(2);
        Ang=0;
        AngleSensorRoomba(serPort);
        while abs(Ang)<2*pi/5
            SetFwdVelAngVelCreate(serPort,0,-0.2);
            pause(0.01);
            Ang=Ang+AngleSensorRoomba(serPort);
        end
        SetFwdVelAngVelCreate(serPort,0.25,0);
    end

    %keep looking until blob large enough - close enough to knocks
	while pixel_count<=480*640*.3
        
        % check if we bump when we shouldnt have
        [right, left, front] = bumpCheckReact(serPort);
        if left
            SetFwdVelAngVelCreate(serPort,-0.25,0);
            pause(1)
            Ang=0;
            AngleSensorRoomba(serPort);
            while abs(Ang)<pi/4
                SetFwdVelAngVelCreate(serPort,0,-0.2);
                pause(0.01);
                Ang=Ang+AngleSensorRoomba(serPort);
            end
            SetFwdVelAngVelCreate(serPort,0,0);
            SetFwdVelAngVelCreate(serPort,0.25,0);
            pause(2)
            Ang=0;
            AngleSensorRoomba(serPort);
            while abs(Ang)<pi/2
                SetFwdVelAngVelCreate(serPort,0,0.2);
                pause(0.01);
                Ang=Ang+AngleSensorRoomba(serPort);
            end
            SetFwdVelAngVelCreate(serPort,0,0);
        elseif right
            SetFwdVelAngVelCreate(serPort,-0.25,0);
            pause(1)
            Ang=0;
            AngleSensorRoomba(serPort);
            while abs(Ang)<pi/4
                SetFwdVelAngVelCreate(serPort,0,0.2);
                pause(0.01);
                Ang=Ang+AngleSensorRoomba(serPort);
            end
            SetFwdVelAngVelCreate(serPort,0,0);
            SetFwdVelAngVelCreate(serPort,0.25,0);
            pause(2)
            Ang=0;
            AngleSensorRoomba(serPort);
            while abs(Ang)<pi/2
                SetFwdVelAngVelCreate(serPort,0,-0.2);
                pause(0.01);
                Ang=Ang+AngleSensorRoomba(serPort);
            end
            SetFwdVelAngVelCreate(serPort,0,0);
        elseif front
            SetFwdVelAngVelCreate(serPort,-0.25,0);
            pause(4)
        end
            
        
		[pic, centroid, pixel_count] = handle_image(pic, colors);
		centroid_x = centroid(2);
		pic_center = size(pic)/2;
		pic_center_x = pic_center(2);
        
        %adjust correction to door
        if flagleft==1
		   GetDoor(serPort,centroid_x,pic_center_x/1.5);
        else
           GetDoor(serPort,centroid_x,2*pic_center_x/1.5);
        end
        
    end
    knock_knock(serPort);
end

%unused
function hw5_part3(serPort)
    Spiral(serPort);
	% take image
     idealcolors = ...
    [0.5618    0.2651    0.4215; ...
     0.5626    0.2660    0.4255; ...
     0.5632    0.2677    0.4297; ...
     0.5640    0.2700    0.4341; ...
     0.5649    0.2722    0.4386];
    global cam_url;

    colors = idealcolors;

	% we now have determined which color we will be tracking - we can start taking live images	
	[initial_pic, initial_centroid, initial_pixel_count] = handle_image(pic, colors);
    disp(initial_pixel_count);
    [pic, centroid, pixel_count] = handle_image(pic, colors);
	centroid_x = centroid(2);
	pic_center = size(pic)/2;
	pic_center_x = pic_center(2);
    GetToDoor(serPort,centroid_x,pic_center_x);
end

%filter image, get centroid, pixel count
function [pic, centroid, pixel_count] = handle_image(pic, colors)

    	global cam_url;

	% take 1st live image
	pic = imread(cam_url);

	% convert rgb2hsv
	pic = rgb2hsv(pic);


	%smooth image using provided gaussian filter
	smoothed_image = gaussfilt(pic,3);
	smoothed_image = decorrstretch(smoothed_image);
	%imshow(smoothed_image);

	% apply filter    
	hue_threshold = .2; % threshold for filter
	saturation_threshold = 0.2;
    	value_threshold = 0.2;
    	n = size(colors,1)*3;
    	filtered_image = 0;
    	i = 1;
    	%disp(colors);
    	while i <= n
        		filtered_image = filtered_image | (smoothed_image(:,:,1) < (colors(i)+hue_threshold) & smoothed_image(:,:,1) > (colors(i)-hue_threshold) & ...
				smoothed_image(:,:,2) < (colors(i+1)+saturation_threshold) & smoothed_image(:,:,2) > (colors(i+1)-saturation_threshold) & ...
				smoothed_image(:,:,3) < (colors(i+2)+value_threshold) & smoothed_image(:,:,3) > (colors(i+2)-value_threshold));
	        	i = i + 3;
    	end
	imshow(filtered_image);
	% erode and dilate
	%se = strel('rectangle', [5,5]);  %square
	se2 = strel('disk', 5); %circle
	eroded_image = imerode(filtered_image, se2);

	dilated_image = imdilate(filtered_image, se2);

	% detect blob
	dilated_image_size = size(dilated_image);
	dilated_image_size_height = dilated_image_size(1);
	dilated_image_size_width = dilated_image_size(2);

	[blob, pixel_count] = get_biggest_blob(dilated_image);
    
    centroid = [0,0];
	for y = 1:dilated_image_size_height
        for x = 1:dilated_image_size_width
            if (blob(y,x)>0)
	 			centroid = centroid + [y,x];
            end
        end
	end

	centroid = centroid / pixel_count;
	pic = dilated_image;
end

% get biggest blob from largest connected component
function [blob, pixel_count] = get_biggest_blob(pic)

	connected_components_pic = bwlabel(pic);
    %disp(connected_components_pic);
	component_count = max(unique(connected_components_pic));
	largest_pixel_count = 0;	
    %disp('comp count');
    %disp(component_count);
    

	% cycle through components detected
	for i = 1:component_count
		% get pixels of current component
		component = (connected_components_pic==i);
		component_pixel_count_not_normalized = sum(component(:));
            %disp('component_pixel_count_not_normalized');
        %disp(component_pixel_count_not_normalized);
        
		component_pixel_count_normalized = component_pixel_count_not_normalized/i;
                    %disp('component_pixel_count_normalized');

        %disp(component_pixel_count_normalized);
		if (component_pixel_count_normalized > largest_pixel_count)
			pixel_count = component_pixel_count_normalized;
            largest_pixel_count = pixel_count;
			blob = component;
            %disp(i);
            %disp(pixel_count);
            %disp(blob);
		end
	end

end

% GAUSSFILT -  Small wrapper function for convenient Gaussian filtering
%
% Usage:  smim = gaussfilt(im, sigma)
%
% Arguments:   im - Image to be smoothed.
%           sigma - Standard deviation of Gaussian filter.
%
% Returns:   smim - Smoothed image.
%
% See also:  INTEGGAUSSFILT
% Peter Kovesi
% Centre for Explortion Targeting
% The University of Western Australia
% http://www.csse.uwa.edu.au/~pk/research/matlabfns/
% March 2010
% Modified to handle HSV values

function smim = gaussfilt(im, sigma)

    % If needed convert im to double
    if ~strcmp(class(im),'double')
        im(:,:,1) = double(im(:,:,1));
        im(:,:,2) = double(im(:,:,2));
        im(:,:,3) = double(im(:,:,3));
    end

    sze = ceil(6*sigma);
    if ~mod(sze,2)    % Ensure filter size is odd
        sze = sze+1;
    end
    sze = max(sze,1); % and make sure it is at least 1
    
    h = fspecial('gaussian', [sze sze], sigma);

    smim(:,:,1) = filter2(h, im(:,:,1));
    smim(:,:,2) = filter2(h, im(:,:,2));
    smim(:,:,3) = filter2(h, im(:,:,3));

end

% help function part 1 - move roomba based on change of color tracking
% object
function adjust_roomba(serPort, sizeDiff, centerDiff)

	maxFwdVel=0.1;
	maxAngVel=0.2;
	sizeThreshold=0.15;
	centerThreshold=0.15;

    %disp(sizeDiff);
    
    if abs(centerDiff)>=centerThreshold
        AngVel=maxAngVel*(centerDiff)*(centerDiff>=-0.5&&centerDiff<=0.5)+...
            maxAngVel*sign(centerDiff)*(centerDiff<-0.5||centerDiff>0.5);
        if abs(AngVel) < 0.1
            AngVel = sign(centerDiff)*.1;
        end
        if abs(AngVel) > maxAngVel
            AngVel = sign(centerDiff)*maxAngVel;
        end
    else
        AngVel = 0;
    end
	%disp(centerDiff);
    %AngVel = AngVel*-1;
    
	if abs(sizeDiff)>=sizeThreshold
		FwdVel=maxFwdVel*((sizeDiff<0)*(sizeDiff/2)+(sizeDiff>=0)*(2*sizeDiff))*(sizeDiff>=-2&&sizeDiff<=0.5)+ ...
		maxFwdVel*sign(sizeDiff)*(sizeDiff<-2||sizeDiff>0.5);  
        if abs(FwdVel) < 0.05
            FwdVel = sign(sizeDiff)*0.05;
        end
        if abs(FwdVel) > maxFwdVel
            FwdVel = sign(sizeDiff)*maxFwdVel;
        end
	else
		FwdVel=0;
    end
        %FwdVel = FwdVel*-1;

    
    
	SetFwdVelAngVelCreate(serPort,FwdVel,AngVel);
	pause(0.01);
end

% simple knock knock function
function knock_knock(serPort)

    maxFwdVel = 0.2;
    SetFwdVelAngVelCreate(serPort, maxFwdVel, 0);

	%drive up to door and bump, then stop
	[right, left, front] = bumpCheckReact(serPort);
	while ~right && ~left && ~front
		[right, left, front] = bumpCheckReact(serPort);
	end
	SetFwdVelAngVelCreate(serPort,0,0);

	%back up
	SetFwdVelAngVelCreate(serPort, -maxFwdVel, 0);
	pause(.5);
    
    SetFwdVelAngVelCreate(serPort, maxFwdVel, 0);


	%bump again
	[right, left, front] = bumpCheckReact(serPort);
	while ~right && ~left && ~front
		[right, left, front] = bumpCheckReact(serPort);
	end
	SetFwdVelAngVelCreate(serPort,0,0);

	%back up
	SetFwdVelAngVelCreate(serPort, -maxFwdVel, 0);
	pause(.5);
    SetFwdVelAngVelCreate(serPort, 0, 0);
	%beep and enter
	BeepRoomba(serPort);
	pause(2);
	SetFwdVelAngVelCreate(serPort, maxFwdVel, 0);
	pause(10);

	%stop roomba - done
	SetFwdVelAngVelCreate(serPort,0,0);

end

% unused
function GetToDoor(serPort,centroid,center)
   FwdVel=0.3;
   AngVel=0.2;  
   if (centroid-center)<0
       direction=1;
   else
       direction=-1;
   end
   dist=1/abs(center-centroid)*center/(1/2);
   %disp(dist)
   dist_temp=0;
   DistanceSensorRoomba(serPort);
   AngleSensorRoomba(serPort);
   %Ang=0;
   while abs(dist_temp-dist)>0.1
       SetFwdVelAngVelCreate(serPort,FwdVel,0);
       pause(0.01);
       dist_temp=dist_temp+DistanceSensorRoomba(serPort);
   end
   SetFwdVelAngVelCreate(serPort,0,0);
   %dist_temp=0;
   DistanceSensorRoomba(serPort);
   AngleSensorRoomba(serPort);
   Ang=0;
   while abs(Ang)<pi/2
       disp('Ang')
       disp(Ang)
       SetFwdVelAngVelCreate(serPort,0,direction*AngVel);
       pause(0.01);
       Ang=Ang+AngleSensorRoomba(serPort);
   end
   SetFwdVelAngVelCreate(serPort,0,0);
end

function GetDoor(serPort,centroid,center)
   FwdVel=0.2;
   AngVel=0.2;
   disp(centroid/center)
   if (centroid-center)<0
       AngleSensorRoomba(serPort);
       Ang=0;
       while abs(Ang)<10/180*pi
          SetFwdVelAngVelCreate(serPort,0,AngVel);
          pause(0.01);
          Ang=Ang+AngleSensorRoomba(serPort);
       end
       SetFwdVelAngVelCreate(serPort,0,0);
   
   
   else
       AngleSensorRoomba(serPort);
       Ang=0;
       while abs(Ang)<10/180*pi
          SetFwdVelAngVelCreate(serPort,0,-AngVel);
          pause(0.01);
          Ang=Ang+AngleSensorRoomba(serPort);
       end
       SetFwdVelAngVelCreate(serPort,0,0);
   end
   SetFwdVelAngVelCreate(serPort,FwdVel,0);
   pause(1);
   SetFwdVelAngVelCreate(serPort,0,0);
end

% first spiral to get close to center of hallway
function Spiral(serPort)
    % Set constants for this program
    AngVel=0.3;
    FwdVel=0.15;
    Width=1.7;
    maxDuration= 1200;  % Max time to allow the program to run (s)
    maxDistSansBump= 5; % Max distance to travel without obstacles (m)
    maxFwdVel= 0.4;     % Max allowable forward velocity with no angular 
                        % velocity at the time (m/s)
    maxVelIncr= 0.005;  % Max incrementation of forward velocity (m/s)
    maxOdomAng= pi/4;   % Max angle to move around a circle before 
                        % increasing the turning radius (rad)
    
    
    % Initialize loop variables
    tStart= tic;        % Time limit marker
    distSansBump= 0;    % Distance traveled without hitting obstacles (m)
    angTurned= 0;       % Angle turned since turning radius increase (rad)
    v= 0;               % Forward velocity (m/s)
    w= v2w(v);          % Angular velocity (rad/s)
    
    % Start robot moving
    SetFwdVelAngVelCreate(serPort,v,w)
    
    % Enter main loop
    while toc(tStart) < maxDuration
        % Check for and react to bump sensor readings
        [right, left, front] = bumpCheckReact(serPort);
        bumped=front;
        
        % If obstacle was hit reset distance and angle recorders
        if bumped
            SetFwdVelAngVelCreate(serPort,0,0);
            break
        end
        
        % Update distance and angle recorders from odometry
        angTurned= angTurned+AngleSensorRoomba(serPort);
        
        % Increase turning radius if it is time
        if angTurned >= maxOdomAng
            % Either increase forward velocity by the increment or by half
            % the difference to the max velocity, whichever is lower
            v= min(v+maxVelIncr,v+(maxFwdVel-v)/2);
            % Choose angular velocity based on max allowable wheel speed
            w= v2w(v);
            SetFwdVelAngVelCreate(serPort,v,w);
            % This could be accomplished more simply by using
            % SetFwdVelRadiusRoomba, this way is just done more fun
        end
        
        % Briefly pause to avoid continuous loop iteration
        pause(0.1);
    end
    
    %turn 180 degrees
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);
    dist=0;
    Ang=0;
    while abs(Ang)<pi
        SetFwdVelAngVelCreate(serPort,0,AngVel);
        pause(0.01);
        Ang=Ang+AngleSensorRoomba(serPort);
    end
    SetFwdVelAngVelCreate(serPort,0,0);
    
    %travel until bumpping, get the distance 
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);
    bumped=0;
    while ~bumped
        [right, left, front] = bumpCheckReact(serPort);
        bumped=right||left||front;
        SetFwdVelAngVelCreate(serPort,FwdVel,0);
        pause(0.01);
        dist=dist+DistanceSensorRoomba(serPort);
    end
    SetFwdVelAngVelCreate(serPort,0,0);
    
    %turn 180 degress
    theta=acos(Width/dist);
    disp('theta')
    disp(theta)
    AngleSensorRoomba(serPort);
    Ang=0;
    while abs(Ang)<pi/2
        SetFwdVelAngVelCreate(serPort,0,AngVel);
        pause(0.01);
        Ang=Ang+AngleSensorRoomba(serPort);
    end
    SetFwdVelAngVelCreate(serPort,0,0);
    AngleSensorRoomba(serPort);
    Ang=0;
    while abs(Ang)<pi/2
        SetFwdVelAngVelCreate(serPort,0,AngVel);
        pause(0.01);
        Ang=Ang+AngleSensorRoomba(serPort);
    end
    SetFwdVelAngVelCreate(serPort,0,0);
    AngleSensorRoomba(serPort);
    Ang=0;
    while abs(Ang)<5*pi/12
        SetFwdVelAngVelCreate(serPort,0,AngVel);
        pause(0.01);
        Ang=Ang+AngleSensorRoomba(serPort);
    end
    SetFwdVelAngVelCreate(serPort,0,0);
    AngleSensorRoomba(serPort);
    Ang=0;
    while abs(Ang)<theta
        SetFwdVelAngVelCreate(serPort,0,AngVel);
        pause(0.01);
        Ang=Ang+AngleSensorRoomba(serPort);
    end
    SetFwdVelAngVelCreate(serPort,0,0);
    disp('turn theta') 
    %try turn theta
    
    SetFwdVelAngVelCreate(serPort,0,0);
    bumped=0;
    dist=0;
    while ~bumped && dist<=0.1
        [right, left, front] = bumpCheckReact(serPort);
        bumped=right||left||front;
        SetFwdVelAngVelCreate(serPort,FwdVel,0);
        pause(0.01);
        dist=dist+DistanceSensorRoomba(serPort);
    end
    SetFwdVelAngVelCreate(serPort,0,0);
    Ang=0;
    AngleSensorRoomba(serPort);
    if ~bumped 
       while abs(Ang)<5*pi/12
          SetFwdVelAngVelCreate(serPort,0,-AngVel);
          pause(0.01);
          Ang=Ang+AngleSensorRoomba(serPort);
          disp('Ang')
          disp(Ang)
       end
    else
       Ang=0;
       AngleSensorRoomba(serPort);
       while abs(Ang)<pi/2
          SetFwdVelAngVelCreate(serPort,0,-AngVel);
          pause(0.01);
          Ang=Ang+AngleSensorRoomba(serPort);
          disp('Ang')
          disp(Ang)
       end 
       SetFwdVelAngVelCreate(serPort,0,0);
       Ang=0;
       AngleSensorRoomba(serPort);
       while abs(Ang)<2*theta
          SetFwdVelAngVelCreate(serPort,0,-AngVel);
          pause(0.01);
          Ang=Ang+AngleSensorRoomba(serPort);
          disp('Ang')
          disp(Ang)
       end 
       
    end
    SetFwdVelAngVelCreate(serPort,0,0);
    %travel half of the Width
    DistanceSensorRoomba(serPort);
    dist1=0;
    while abs(dist1-Width/2)>0.05
       SetFwdVelAngVelCreate(serPort,FwdVel,0);
       pause(0.01);
       dist1=dist1+DistanceSensorRoomba(serPort);
    end
    SetFwdVelAngVelCreate(serPort,0,0);
    %turn 90 degrees
    AngleSensorRoomba(serPort);
    Ang=0;
    while Ang<pi/2
        SetFwdVelAngVelCreate(serPort,0,AngVel);
        pause(0.01);
        Ang=Ang+AngleSensorRoomba(serPort);
    end  
    SetFwdVelAngVelCreate(serPort,0,0);
end


function [BumpRight, BumpLeft, BumpFront] = bumpCheckReact(serPort)

    [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);

end


function w= v2w(v)
% Calculate the maximum allowable angular velocity from the linear velocity
%
% Input:
% v - Forward velocity of Create (m/s)
%
% Output:
% w - Angular velocity of Create (rad/s)
    
    % Robot constants
    maxWheelVel= 0.5;   % Max linear velocity of each drive wheel (m/s)
    robotRadius= 0.2;   % Radius of the robot (m)
    
    % Max velocity combinations obey rule v+wr <= v_max
    w= (maxWheelVel-v)/robotRadius;
end