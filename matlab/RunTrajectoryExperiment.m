function output_data = RunTrajectoryExperiment( angleR1_init, angleR2_init, pts_footR, ...
    angleL1_init, angleL2_init, pts_footL, ...
    traj_time, pre_buffer_time, pre_buffer_timeR, pre_buffer_timeL, post_buffer_time, gains, duty_max)
    
    % Figure for plotting motor data
    figure(1);  clf;       
    a1 = subplot(421);
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Angle 1 (rad)');
    
    a2 = subplot(423);
    h2 = plot([0],[0]);
    h2.XData = []; h2.YData = [];
    ylabel('Velocity 1 (rad/s)');
    
    a3 = subplot(425);
    h3 = plot([0],[0]);
    h3.XData = []; h3.YData = [];
    ylabel('Current 1 (A)');
    hold on;
    subplot(425);
    h4 = plot([0],[0],'r');
    h4.XData = []; h4.YData = [];
    hold off;
    
    a4 = subplot(427);
    h5 = plot([0],[0]);
    h5.XData = []; h5.YData = [];
    ylabel('Duty Cycle 1');
    

    a5 = subplot(422);
    h21 = plot([0],[0]);
    h21.XData = []; h21.YData = [];
    ylabel('Angle 2 (rad)');
    
    a6 = subplot(424);
    h22 = plot([0],[0]);
    h22.XData = []; h22.YData = [];
    ylabel('Velocity 2 (rad/s)');
    
    a7 = subplot(426);
    h23 = plot([0],[0]);
    h23.XData = []; h23.YData = [];
    ylabel('Current 2 (A)');
    hold on;
    subplot(426);
    h24 = plot([0],[0],'r');
    h24.XData = []; h24.YData = [];
    hold off;
    
    a8 = subplot(428);
    h25 = plot([0],[0]);
    h25.XData = []; h25.YData = [];
    ylabel('Duty Cycle 2');
    
    % Figure for plotting state of the leg
    figure(2)
    clf
    hold on
    axis equal
    axis([-.35 .35 -.35 .1]);
   
    h_OB = plot([0],[0],'LineWidth',2);
    h_AC = plot([0],[0],'LineWidth',2);
    h_BD = plot([0],[0],'LineWidth',2);
    h_CE = plot([0],[0],'LineWidth',2);
    h_ellip = plot([0],[0],'g','LineWidth',1.5);
    
    h_foot= plot([0],[0],'Color',[0.7,0.7,0.7]);
    h_des = plot([0],[0],'--','Color',[0.5,0.5,0.5]);
    h_des.XData=[];
    h_des.YData=[];
    h_foot.XData=[];
    h_foot.YData=[];
    
    % Define leg length parameters
    m1 =.0393 + .2;         m2 =.0368; 
    m3 = .00783;            m4 = .0155;
    I1 = 25.1 * 10^-6;      I2 = 53.5 * 10^-6;
    I3 = 9.25 * 10^-6;      I4 = 22.176 * 10^-6;
    l_OA=.011;              l_OB=.042; 
    l_AC=.096;              l_DE=.091;
    l_O_m1=0.032;           l_B_m2=0.0344; 
    l_A_m3=0.0622;          l_C_m4=0.0610;
    Nmot = 18.75;
    Ir = 0.0035/Nmot^2;

    p   = [l_OA l_OB l_AC l_DE];
    p   = [p m1 m2 m3 m4 I1 I2 I3 I4 Ir Nmot l_O_m1 l_B_m2 l_A_m3 l_C_m4]';
    
    % This function will get called any time there is new data from
    % the Nucleo board. Data comes in blocks, rather than one at a time.
    function my_callback(new_data)
        % Parse new data
        t = new_data(:,1);          % time
        posR1 = new_data(:,2);       % position
        velR1 = new_data(:,3);       % velocity
        curR1 = new_data(:,4);       % current
        dcurR1 = new_data(:,5);      % desired current
        dutyR1 = new_data(:,6);      % command
        
        posR2 = new_data(:,7);       % position
        velR2 = new_data(:,8);       % velocity
        curR2 = new_data(:,9);       % current
        dcurR2 = new_data(:,10);     % desired current
        dutyR2 = new_data(:,11);     % command

        xR = -new_data(:,12);         % actual foot position (negative due to direction motors are mounted)
        yR = new_data(:,13);         % actual foot position
        %speed missing 14
        %speed missing 15
        xRdes = -new_data(:,16);      % desired foot position (negative due to direction motors are mounted)
        yRdes = new_data(:,17);      % desired foot position  
        dxRdes = new_data(:,18);
        dyRDes = new_data(:,19);

        posL1 = new_data(:,20);       % position
        velL1 = new_data(:,21);       % velocity
        curL1 = new_data(:,22);       % current
        dcurL1 = new_data(:,23);      % desired current
        dutyL1 = new_data(:,24);      % command
        
        posL2 = new_data(:,25);       % position
        velL2 = new_data(:,26);       % velocity
        curL2 = new_data(:,27);       % current
        dcurL2 = new_data(:,28);     % desired current
        dutyL2 = new_data(:,29);     % command
        
        xL = -new_data(:,30);         % actual foot position (negative due to direction motors are mounted)
        yL = new_data(:,31);         % actual foot position
        % speed missing
        % speed missing
        xLdes = -new_data(:,34);      % desired foot position (negative due to direction motors are mounted)
        yLdes = new_data(:,35);      % desired foot position         
        dxLdes = new_data(:,36);
        dyLdes = new_data(:,37);
    

        N = length(posR1);
        
        % Update motor data plots
        h1.XData(end+1:end+N) = t;   
        h1.YData(end+1:end+N) = -posR1; % switch sign on all plotted values due to direction motors are mounted
        h2.XData(end+1:end+N) = t;   
        h2.YData(end+1:end+N) = -velR1;
        h3.XData(end+1:end+N) = t;   
        h3.YData(end+1:end+N) = -curR1;
        h4.XData(end+1:end+N) = t;   
        h4.YData(end+1:end+N) = -dcurR1;
        h5.XData(end+1:end+N) = t;   
        h5.YData(end+1:end+N) = -dutyR1;
        
        h21.XData(end+1:end+N) = t;   
        h21.YData(end+1:end+N) = -posR2;
        h22.XData(end+1:end+N) = t;   
        h22.YData(end+1:end+N) = -velR2;
        h23.XData(end+1:end+N) = t;   
        h23.YData(end+1:end+N) = -curR2;
        h24.XData(end+1:end+N) = t;   
        h24.YData(end+1:end+N) = -dcurR2;
        h25.XData(end+1:end+N) = t;   
        h25.YData(end+1:end+N) = -dutyR2;
        
        % Calculate leg state and update plots
        zR = [posR1(end) posR2(end) velR1(end) velR2(end)]';
        keypoints = keypoints_leg(zR,p);
        inertia_ellipse = inertia_ellipse_leg(zR,p);
        
        % TODO: could also plot Jacobian, control force vector here?
        
        rR_A = keypoints(:,1); 
        rR_B = keypoints(:,2);
        rR_C = keypoints(:,3);
        rR_D = keypoints(:,4);
        rR_E = keypoints(:,5);

        set(h_OB,'XData',[0 rR_B(1)],'YData',[0 rR_B(2)]);
        set(h_AC,'XData',[rR_A(1) rR_C(1)],'YData',[rR_A(2) rR_C(2)]);
        set(h_BD,'XData',[rR_B(1) rR_D(1)],'YData',[rR_B(2) rR_D(2)]);
        set(h_CE,'XData',[rR_C(1) rR_E(1)],'YData',[rR_C(2) rR_E(2)]);
        
        ellipse_x = inertia_ellipse(1,:) + rR_E(1);
        ellipse_y = inertia_ellipse(2,:) + rR_E(2);
        set(h_ellip,'XData',ellipse_x,'YData',ellipse_y);
        
        h_foot.XData(end+1:end+N) = xR;
        h_foot.YData(end+1:end+N) = yR;
        h_des.XData(end+1:end+N) = xRdes;
        h_des.YData(end+1:end+N) = yRdes;
        
    end
    
    frdm_ip  = '192.168.1.100';     % FRDM board ip
    frdm_port= 11223;               % FRDM board port  
    params.callback = @my_callback; % callback function
    %params.timeout  = 2;            % end of experiment timeout
    
    % Parameters for tuning
    start_period                 = pre_buffer_time; 
    start_periodR                = pre_buffer_timeR;    % In seconds 
    start_periodL                = pre_buffer_timeL;    % In seconds 
    end_period                  = post_buffer_time;   % In seconds
    
    K_xx                     = gains.K_xx; % Stiffness
    K_yy                     = gains.K_yy; % Stiffness
    K_xy                     = gains.K_xy; % Stiffness

    D_xx                     = gains.D_xx; % Damping
    D_yy                     = gains.D_yy; % Damping
    D_xy                     = gains.D_xy; % Damping
    
    % Specify inputs
    input = [start_period start_periodR start_periodL traj_time end_period];
    input = [input angleR1_init angleR2_init angleL1_init angleL2_init];
    input = [input K_xx K_yy K_xy D_xx D_yy D_xy];
    input = [input duty_max];
    input = [input pts_footR(:)' pts_footL(:)']; % final size of input should be 28x1
    
    params.timeout  = (start_period+traj_time+end_period);  
    
    output_size = 37;    % number of outputs expected
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
    linkaxes([a1 a2 a3 a4],'x')
    
end