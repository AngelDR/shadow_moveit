function newHand = ShadowHand(T)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    SHADOW   H A N D    P A R A M E T E R S
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Pre-allocation
DHpars{5} = [];
base{5} = [];
F{5} = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Thumb

% DHpars{1}=[
%     0 0 0 -2.35 ;            
%     0 0 0 pi/2 ;         
%     0 0 38 0;
%     0 0 0 -pi/2;
%     0 0 32 0;
%     0 0 27.5 0];  

DHpars{1}=[
    0 pi/2 0 0 ;            
    0 0 0 -pi/2 ;         
    0 -pi/2 38 0;
    0 0 0 -pi/2;
    0 0 32 0;
    0 0 27.5 0];  


DHpars{1} = [DHpars{1}(:,4) DHpars{1}(:,3) DHpars{1}(:,2) DHpars{1}(:,1)];


base{1} = [ 0.707 0 0.707 34
0 1 0 -8.5
-0.707 0 0.707 29
0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Index
DHpars{2}=[
    0 0 0 0 ;            
    0 pi/2 0 -pi/2 ;         
    0 0 45 0;
    0 0 25 0;
    0 0 26 0];     

DHpars{2} = [DHpars{2}(:,4) DHpars{2}(:,3) DHpars{2}(:,2) DHpars{2}(:,1)];

base{2} = [1 0 0 33;
    0 0 -1 0;
    0 1 0 95;
    0 0 0 1];


%%%%%%%%%%%%%%%%%%%%%%%%%%%% Middle
DHpars{3}=[
    0 0 0 0 ;            
    0 pi/2 0 -pi/2 ;         
    0 0 45 0;
    0 0 25 0;
    0 0 26 0];  
DHpars{3} = [DHpars{3}(:,4) DHpars{3}(:,3) DHpars{3}(:,2) DHpars{3}(:,1)];

base{3} = [1 0 0 11;
    0 0 -1 0;
    0 1 0 99;
    0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%% Ring
DHpars{4}=[
    0 0 0 0 ;            
    0 pi/2 0 -pi/2 ;         
    0 0 45 0;
    0 0 25 0;
    0 0 26 0]; 

DHpars{4} = [DHpars{4}(:,4) DHpars{4}(:,3) DHpars{4}(:,2) DHpars{4}(:,1)];

base{4} = [1 0 0 -11;
    0 0 -1 0;
    0 1 0 95;
    0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%% Pinkie  % J5
% DHpars{5}=[
%     0 0 0 0;            
%     52.663 0 -36.875 -pi/2;         
%     0 -2.181 0 pi/2;
%     0 0 45 0;
%     0 0 25 0;
%     0 0 26 0]; 
% 
% DHpars{5} = [DHpars{5}(:,4) DHpars{5}(:,3) DHpars{5}(:,2) DHpars{5}(:,1)];
% 
% base{5} = [0.819 0 0.573  -33;
%     0 1 0 0;
%     -0.573 0 0.189 20.71;
%     0 0 0 1];


%%%%%%%%%%%%%%%%%%%%%%%%%% Pinkie  % J4
DHpars{5}=[
    0 0 0 0
    70 0 0 pi/2 ;                   
    0 pi/2 0 -pi/2 ;         
    0 0 45 0;
    0 0 25 0;
    0 0 26 0]; 

DHpars{5} = [DHpars{5}(:,4) DHpars{5}(:,3) DHpars{5}(:,2) DHpars{5}(:,1)];


%% A partir de transformation matrix (lfj4 -> palm)
base{5} = [1 0 0  -33;
    0 1 0 0;
    0 0 1 20;
    0 0 0 1];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


for i = 1:length(DHpars)
    % number of joints for each finger
    joints = size(DHpars{i},1);
    % initialize joint variables
    q = zeros(joints,1);
    % make the finger
    F{i} = SGmakeFinger(DHpars{i},base{i},q);
end

newHand=SGmakeHand(F);
newHand.type = 'Shadow';



