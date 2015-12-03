hand = ShadowHand;
figure(1);
% 
% q_zero = [0.0,1.57,-1.57,0.0,-0.0,-0.0,...
%       0.0,1.57,-0.0,-0.0,-0.0,...
%       0.0,1.57,-0.0,-0.0,-0.0,...
%       0.0,1.57,-0.0,-0.0,-0.0,...
%       0.0,0.0,1.57,-0.,-0.0,-0.0];
% hand = SGmoveHand(hand,q_zero);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%q_limits                                             
%articulacion: limites (grados) (limites (rad)) -> [limites_modelo (rad)]


% first / middle / ring / little:                     
%     j4 : -20º-> 20º  (-0.349 - 0.349)  -> [1.23 - 1.91]             
%     j3 : 0º -> 90º (0.0 - 1.57) -> [0.0 - -1.57]                    
%     j0(j1,j2)-> 0 -> 180º (0.0 - 3.1415) -> [0.0 - -3.14]           
%     

% little:                                             
%     j5: 0º -> 45º  (0 - 0.785)                      
% 

% thumb:                                              
%     j5: -60º -> 60º  (-1.047 - 1.047) -> (-0.53 -> 2.61)              
%     j4: 0º -> 70º (0.0 - 1.221)  -> (-1.57 -> -2.79)                     
%     j3: -15º -> 15º (-0.261 - 0.261) -> (-0.261 - 0.261)                
%     j2: -40º -> 40º (-0.698 - 0.698) -> (-0.698 - 0.698)              
%     j1: 0º -> 90º (0.0 -  1.57 ) -> (0.0 -  -1.57 )                    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  
qm = [0.0,1.57,-2.79,-0.26,-0.5,-0.5,...
  0.0,1.37,-1.3,-0.5,-0.5,...
  0.0,1.47,-1.3,-0.5,-0.5,...
  0.0,1.67,-1.3,-0.5,-0.5,...
  0.0,0.0,1.77,-1.3,-0.5,-0.5];
hand = SGmoveHand(hand,qm);

%Definir contactos
hand = SGaddContact(hand,2,2,5,1);
hand = SGaddContact(hand,2,3,5,1);
hand = SGaddContact(hand,2,4,5,1);
hand = SGaddContact(hand,2,5,6,1);
hand = SGaddContact(hand,2,1,6,1);

%Definir objeto
[hand,object] = SGmakeObject(hand);

% Plot
SGplotObject(object);

SGplotHand(hand);
axis equal;
box off;
grid off;


for i=1:10
    qm = [0.0,1.57,-2.79,-0.26,-0.5,-0.5+i*(0.05),...
    0.0,1.37,-1.3+i*(0.05),-0.5,-0.5,...
    0.0,1.47,-1.3+i*(0.05),-0.5,-0.5,...
    0.0,1.67,-1.3+i*(0.05),-0.5,-0.5,...
    0.0,0.0,1.77,-1.3+i*(0.05),-0.5,-0.5]; 
    hand = SGmoveHand(hand,qm);
    %%%%%%refreshdata(1);
    [hand,object] = SGmakeObject(hand);
    SGplotObject(object);
    SGplotHand(hand);
    axis equal;
    box off;
    grid off;
    pause(1);
end

%  Grasp Matrix
hand.G = SGgraspMatrix(object);
hand.J = SGjacobianMatrix(hand);
disp(hand.G);
disp(hand.J);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%  GRASP ANALYSIS %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Grasp analisis
[E,V,N] = SGgraspAnalysis(hand);

% Grasp stiffness
%K = SGgraspStiffness(hand, object);

% Rigid body motion
% rbmotion = SGrbMotions(hand,object);
% disp(rbmotion);

% Quasi static maps
% linMaps = SGquasistaticMaps(hand,object)

% Quasi static solution
%SGquasistaticHsolution(hand, object);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% GRASP QUALITY  MEASURES %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[SminHO] = SGdistSingularConfiguration(hand.G,hand.J);

[Ii] = SGgraspIsotropyIndex (hand.G);

[VE] = SGmanipEllipsoidVolume(hand.G,hand.J);

[SminG] = SGminSVG(hand.G);

[Ut] = SGunifTransf(hand.G,hand.J);




