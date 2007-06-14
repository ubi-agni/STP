% NEW open problems with streched profiles
if (true)    
end

% working stretched profile cases
bPlot = false;
if (true)
   % WT, ddec ==> WW, ddec
   [t,j] = calc7st(4.228053, 0.857550, 1.727873, 3.105438, 0.2, 5, -10.239185);
   stretch7st(t,j,6,4.228053, 0.857550, 1.727873, 3.105438, 0.2, 5, -10.239185,bPlot);

   % WcT ==> ddec
   [t,j] = calc7st(5,2,2,3,0,2,-5);
   stretch7st(t,j,6.5,5,2,2,3,0,2,-5,bPlot);
    
   % TcT, a0 ==> ddec ??
   [t,j] = calc7st(3,2,1.5,2,3,-2,-3);
   stretch7st(t,j,30,3,2,1.5,2,3,-2,-3,bPlot);

   % WcW, ddec ==> TcW, ddec
   [t,j] = calc7st(4,2,2,2,2.5,1,-4);
   stretch7st(t,j,10,4,2,2,2,2.5,1,-4, bPlot);

   % TcT ==> TcT
   [t,j] = calc7st(5,2,2,3,0,0,-5);
   stretch7st(t,j,6.5,5,2,2,3,0,0,-5,bPlot);
   
   % TcT, a0 ==> TcW, a0
   [t,j] = calc7st(3,2,1.5,2,3,-2,-3);
   stretch7st(t,j,10,3,2,1.5,2,3,-2,-3,bPlot);
   
   % WcW, ddec ==> WcW, ddec
   [t,j] = calc7st(4,2,2,2,2.5,1,-4);
   stretch7st(t,j,5,4,2,2,2,2.5,1,-4, bPlot);
   
   % WcT ==> WcT
   [t,j] = calc7st(5,2,2,3,0,2,-5);
   stretch7st(t,j,5,5,2,2,3,0,2,-5,bPlot);
   
   % TT ==> WcW
   [t,j] = calc7st(4,4,3,4,0,0,-4);
   stretch7st(t,j,6,4,4,3,4,0,0,-4,bPlot);
   
   % WW ==> WcW
   [t,j] = calc7st(4,1,2,3,0,0,-4);
   stretch7st(t,j,7,4,1,2,3,0,0,-4,bPlot);
   
   % TW, v0 ==> TcW, v0
   [t,j] = calc7st(10, 4, 8, 12, -5, 15, 0);
   stretch7st(t, j, 5.5, 10, 4, 8, 12, -5, 15, 0, bPlot);
end

bPlot = false;
% open problems
calc7st (-0.030068, 2,1,1, 0.8,-0.2,0, bPlot, true, true);
calc7st (-0.275, 2,1,1, -0.8,-0.2,0, bPlot, true, true);
calc7st (-0.27, 2,1,1, -0.8,-0.2,0, bPlot, true, true);
% description: negative time intervall t(3) in WW case
calc7st(1.5, 3, 3, 3, -2, 2, 0, bPlot, true, true);

% doesn't reach target / a is too high (if using less precision it works!)
calc7st(0.26400000000000068, 1, 1, 1, -1.2, 0.7, 0, bPlot, true, true);
calc7st(0.11433333333333362, 1, 1, 1, 0.7, 0, 0, bPlot, true, true)
calc7st(-0.11433333333333362, 1, 1, 1, -0.7, 0, 0, bPlot, true, true)
% a is too high
calc7st(0.77973084054605746, 1, 1, 1, -1, 1.2, 0, bPlot, true, true)
calc7st(0.89230768364536051, 1, 1, 1, -0.7, 1.2, 0, bPlot, true, true)
calc7st(1.1003205353432297, 1, 1, 1, -0.7, 1.2, 0, bPlot, true, true)
calc7st(1.3083333870410994, 1, 1, 1, -0.7, 1.2, 0, bPlot, true, true)
calc7st(-1.3083333870410985, 1, 1, 1, 0.7, -1.2, 0, bPlot, true, true)
calc7st(-1.1003205353432293, 1, 1, 1, 0.7, -1.2, 0, bPlot, true, true)
calc7st(-0.89230768364536006, 1, 1, 1, 0.7, -1.2, 0, bPlot, true, true)
calc7st(-0.77973084054605746, 1, 1, 1, 1, -1.2, 0, bPlot, true, true)

% let's start with some easy some standard tests with cruising phase:
if true
    % TcT
    calc7st(5,2,2,3,0,0,-5,false, false, true);

    % TcW
    calc7st(6,1.25,2,3,1.5,-1,-5,false, false, true);

    % WcT
    calc7st(5,2,2,3,0,2,-5,false, false, true);

    % WcW
    calc7st(4,1,2,2,0,0,-4,false, false, true);

    % TcT, a0>amax
    calc7st(3,2,1.5,2,3,-2,-3,false, false, true);

    % DcD v0 will exceed vmax
    calc7st(4,1.5,2,2,1.5,1.5,-4,false, false, true);

    % DcD, a0>amax, v0 will exceed vmax
    calc7st(4,2,2,2,2.5,1,-4,false, false, true);
end

% some more easy cases, but now without cruising phase:
if true
    % WW, v0>vmax
    calc7st(4.228053, 0.857550, 1.727873, 3.105438, -0.2, 5, -10.239185,false, false, true);

    % WT, v0>vmax
    calc7st(4.228053, 0.857550, 1.727873, 3.105438, 0.2, 5, -10.239185,false, false, true);

    % WW
    calc7st(4,1,2,3,0,0,-4,false, false,true);

    % TT cut out possible
    calc7st(4,4,3,4,0,0,-4,false, false, true);

    % TW
    calc7st(10, 4, 8, 12, -5, 15, 0, false, false, true);
end

% solved problems
% description: producing a WW instead of TW profile so breaking amax
calc7st(1, 4, 4.5, 12, -2, -4, 0,false,true,true);
% description: negative time intervalls at t(1) and t(5).
calc7st(4, 2, 10, 10, 2, -15, 0,false,true,false);

% double deceleration WT
calc7st(4, 3,2,1, 4,0.5, -3, false, false, true);
calc7st(4.228053, 0.857550, 1.727873, 3.105438, 0.2, 5, -10.239185,false, false, true);
calc7st(3.7, 3,2,1, 4,0.5, -3, false, false, true);
