cd %PUMA560 Load kinematic and dynamic data for a Puma 560 manipulator
%
%	PUMA560
%
% Defines the object 'p560' in the current workspace which describes the 
% kinematic and dynamic characterstics of a Unimation Puma 560 manipulator
% using standard DH conventions.
% The model includes armature inertia and gear ratios.
%
% Also define the vector qz which corresponds to the zero joint
% angle configuration, qr which is the vertical 'READY' configuration,
% and qstretch in which the arm is stretched out in the X direction.
%
% See also: ROBOT, PUMA560AKB, STANFORD, TWOLINK.

%
% Notes:
%    - the value of m1 is given as 0 here.  Armstrong found no value for it
% and it does not appear in the equation for tau1 after the substituion
% is made to inertia about link frame rather than COG frame.
% updated:
% 2/8/95  changed D3 to 150.05mm which is closer to data from Lee, AKB86 and Tarn
%  fixed errors in COG for links 2 and 3
% 29/1/91 to agree with data from Armstrong etal.  Due to their use
%  of modified D&H params, some of the offsets Ai, Di are
%  offset, and for links 3-5 swap Y and Z axes.
% 14/2/91 to use Paul's value of link twist (alpha) to be consistant
%  with ARCL.  This is the -ve of Lee's values, which means the
%  zero angle position is a righty for Paul, and lefty for Lee.
%  Note that gravity load torque is the motor torque necessary
%  to keep the joint static, and is thus -ve of the gravity
%  caused torque.
%
% 8/95 fix bugs in COG data for Puma 560. This led to signficant errors in
%  inertia of joint 1. 
% $Log: not supported by cvs2svn $
% Revision 1.4  2008/04/27 11:36:54  cor134
% Add nominal (non singular) pose qn

% Copyright (C) 1993-2008, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.

%A sequencia é alpha, A, theta e D, sigma e offset;
% clear L
% L{1} = link([     0       0       .459    0 0], 'standard');
% L{2} = link([pi/2  .150	0       0       0 0], 'standard');
% L{3} = link([0      .590	0       0       0 0], 'standard');
% L{4} = link([-pi/2  0    0       .647	0 0], 'standard');
% L{5} = link([-pi/2  0       0       0       0 0], 'standard');
% L{6} = link([pi/2   0       0       .095    0 0], 'standard');

clear L
L{1} = link([pi     0       0       -.450   0 0], 'modified');
L{2} = link([pi/2  .150	0       0       0 -pi/2], 'modified');
L{3} = link([pi      .590	0  0         0 -pi/2], 'modified');
L{4} = link([-pi/2  -.130     0 .64707	0 0], 'modified');
L{5} = link([-pi/2  0       0   0  0 pi], 'modified');
L{6} = link([-pi/2   0       0       .095    0 0], 'modified');

% 
% clear L
% L{1} = link([ pi/2 0	0	0	0], 'standard');
% L{2} = link([ 0 	.4318	0	0	0], 'standard');
% L{3} = link([-pi/2 .0203	0	.15005	0], 'standard');
% L{4} = link([pi/2 0	0	.4318	0], 'standard');
% L{5} = link([-pi/2 0	0	0	0], 'standard');
% L{6} = link([0 	0	0	0	0], 'standard');

% L{1} = link([ pi    0       0       .459    0 0], 'standard');
% L{2} = link([ pi/2  .150	0       0       0 -pi/2], 'standard');
% L{3} = link([0      .590	0       0       0 -pi/2], 'standard');
% L{4} = link([-pi/2  .130    0       .647	0 0], 'standard');
% L{5} = link([-pi/2  0       0       0       0 0], 'standard');
% L{6} = link([pi/2   0       0       .095       0 0], 'standard');


L{1}.m = 0;
L{2}.m = 17.4;
L{3}.m = 4.8;
L{4}.m = 0.82;
L{5}.m = 0.34;
L{6}.m = .09;

L{1}.r = [ 0    0	   0 ];
L{2}.r = [ -.3638  .006    .2275];
L{3}.r = [ -.0203  -.0141  .070];
L{4}.r = [ 0    .019    0];
L{5}.r = [ 0    0	   0];
L{6}.r = [ 0    0	   .032];

L{1}.I = [  0	 0.35	 0	 0	 0	 0];
L{2}.I = [  .13	 .524	 .539	 0	 0	 0];
L{3}.I = [   .066  .086	 .0125   0	 0	 0];
L{4}.I = [  1.8e-3  1.3e-3  1.8e-3  0	 0	 0];
L{5}.I = [  .3e-3   .4e-3   .3e-3   0	 0	 0];
L{6}.I = [  .15e-3  .15e-3  .04e-3  0	 0	 0];

L{1}.Jm =  200e-6;
L{2}.Jm =  200e-6;
L{3}.Jm =  200e-6;
L{4}.Jm =  33e-6;
L{5}.Jm =  33e-6;
L{6}.Jm =  33e-6;

L{1}.G =  -62.6111;
L{2}.G =  107.815;
L{3}.G =  -53.7063;
L{4}.G =  76.0364;
L{5}.G =  71.923;
L{6}.G =  76.686;

% viscous friction (motor referenced)
L{1}.B =   1.48e-3;
L{2}.B =   .817e-3;
L{3}.B =    1.38e-3;
L{4}.B =   71.2e-6;
L{5}.B =   82.6e-6;
L{6}.B =   36.7e-6;

% Coulomb friction (motor referenced)
L{1}.Tc = [ .395	-.435];
L{2}.Tc = [ .126	-.071];
L{3}.Tc = [ .132	-.105];
L{4}.Tc = [ 11.2e-3 -16.9e-3];
L{5}.Tc = [ 9.26e-3 -14.5e-3];
L{6}.Tc = [ 3.96e-3 -10.5e-3];


%
% some useful poses
%
% qz = [0 0 0 0 0 0 ]; % zero angles, L shaped pose
% qr = [0 pi/2 -pi/2 0 0 0]; % ready pose, arm up
% qs = [0 0 -pi/2 0 0 0];
% qn=[0 pi/4 pi 0 pi/4 0];

% qz = [0 0 0 0 0 0 ]; % zero angles, L shaped pose
% qr = [0 pi/2 -pi/2 0 0 0]; % ready pose, arm up
% qs = [0 0 -pi/2 0 0 0];
% qn=[0 pi/4 pi 0 pi/4 0];


%p560 = robot(L, 'Puma 560', 'Unimation', 'params of 8/95');
%clear L
%p560.name = 'Puma 560';
%p560.manuf = 'Unimation';

smart6 = robot(L, 'Smart Six', 'COMAU', 'params of 8/95');
clear L
smart6.name = 'Smart Six';
smart6.manuf = 'COMAU';

% Constantes do robô
txrate = 2 * pi;
driveON = 0;

cd ~coro/Adriano/Simulador/mexRtnet/

% Init Packet;
seqNumber 			   = 10;
numberOfFieldsPerPacket = 363;
numberOfFieldsPerAxis   = 9;
numberOfAxesInOpenMode  = 5;

typeOfHeaderField1	= 0;
typeOfHeaderField2 	= 0;
typeOfHeaderField3 	= 0;
typeOfAxisDataField1 = 0;
typeOfAxisDataField2 = 0;
typeOfAxisDataField3 = 0;
typeOfAxisDataField4 = 0;
typeOfAxisDataField5 = 0;
typeOfAxisDataField6 = 0;
typeOfAxisDataField7 = 0;
typeOfAxisDataField8 = 0;
typeOfAxisDataField9 = 0;

sampleTime  = 1500;

majorNumber = 0;
minorNumber = 0;
buildNumber = 0;

arm1OpenMode = [0  -1  0  0  0  0 -1 -1 -1 -1];
arm2OpenMode = [-1 -1 -1 -1 -1 -1 -1 -1 -1 -1];
arm3OpenMode = [-1 -1 -1 -1 -1 -1 -1 -1 -1 -1];
arm4OpenMode = [-1 -1 -1 -1 -1 -1 -1 -1 -1 -1];

arm1OpenAxesMap = [ 1 -1  1  1  1  1 -1 -1 -1 -1];
arm2OpenAxesMap = [-1 -1 -1 -1 -1 -1 -1 -1 -1 -1];
arm3OpenAxesMap = [-1 -1 -1 -1 -1 -1 -1 -1 -1 -1];
arm4OpenAxesMap = [-1 -1 -1 -1 -1 -1 -1 -1 -1 -1];

arm1CalibrationConstant = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];
arm2CalibrationConstant = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];
arm3CalibrationConstant = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];
arm4CalibrationConstant = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];

arm1CurrentLimit = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];
arm2CurrentLimit = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];
arm3CurrentLimit = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];
arm4CurrentLimit = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];

arm1TxRate = [txrate NaN txrate txrate txrate txrate NaN NaN NaN NaN];
arm2TxRate = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];
arm3TxRate = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];
arm4TxRate = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];

arm1KinInflCoeff = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];
arm2KinInflCoeff = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];
arm3KinInflCoeff = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];
arm4KinInflCoeff = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];

arm1FollowingError = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];
arm2FollowingError = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];
arm3FollowingError = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];
arm4FollowingError = [NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];


% Pacote Comum para envio
SM =    [4 4 4 4 4 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1];    % mode;
D1  =   [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];   % useful data 1;
D2  =   [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];   % useful data 2;
D3  =   [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];   % useful data 3;
D4  =   [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];   % useful data 4;
D5  =   [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];   % useful data 5;
EXT1  = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]; % extra data 1; 
EXT2  = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]; % extra data 2; 
EXT3  = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]; % extra data 3; 

% Pacote recebido
oSM =   [4 4 4 4 4 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1];    % mode;
oD1  =  [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];   % useful data 1;
oD2  =  [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];   % useful data 2;
oD3  =  [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];   % useful data 3;
oD4  =  [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];   % useful data 4;
oD5  =  [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];   % useful data 5;
oEXT1 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]; % extra data 1; 
oEXT2 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]; % extra data 2; 
oEXT3 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]; % extra data 3; 


% Usuário seta variaveis
% --- 

% Escreve o pacote Inicial
fid = fopen('entrada.txt','w');

%fprintf(fid,'%d\n',initPacket.seqNumber);
%fprintf(fid,'%d\n',initPacket.numberOfFieldsPerPacket);
%fprintf(fid,'%d\n',initPacket.numberOfFieldsPerAxis);
fprintf(fid,'%d\n',numberOfAxesInOpenMode);

fprintf(fid,'%d\n',typeOfHeaderField1);
fprintf(fid,'%d\n',typeOfHeaderField2);
fprintf(fid,'%d\n',typeOfHeaderField3);
fprintf(fid,'%d\n',typeOfAxisDataField1);
fprintf(fid,'%d\n',typeOfAxisDataField2);
fprintf(fid,'%d\n',typeOfAxisDataField3);
fprintf(fid,'%d\n',typeOfAxisDataField4);
fprintf(fid,'%d\n',typeOfAxisDataField5);
fprintf(fid,'%d\n',typeOfAxisDataField6);
fprintf(fid,'%d\n',typeOfAxisDataField7);
fprintf(fid,'%d\n',typeOfAxisDataField8);
fprintf(fid,'%d\n',typeOfAxisDataField9);

fprintf(fid,'%d\n',sampleTime);

fprintf(fid,'%d\n',majorNumber);
fprintf(fid,'%d\n',minorNumber);
fprintf(fid,'%d\n',buildNumber);

for i = 1:10
    fprintf(fid,'%d\n',arm1OpenMode(i));
    fprintf(fid,'%d\n',arm2OpenMode(i));
    fprintf(fid,'%d\n',arm3OpenMode(i));
    fprintf(fid,'%d\n',arm4OpenMode(i));

    fprintf(fid,'%d\n',arm1OpenAxesMap(i));
    fprintf(fid,'%d\n',arm2OpenAxesMap(i));
    fprintf(fid,'%d\n',arm3OpenAxesMap(i));
    fprintf(fid,'%d\n',arm4OpenAxesMap(i));

    fprintf(fid,'%d\n',arm1CalibrationConstant(i));
    fprintf(fid,'%d\n',arm2CalibrationConstant(i));
    fprintf(fid,'%d\n',arm3CalibrationConstant(i));
    fprintf(fid,'%d\n',arm4CalibrationConstant(i));

    fprintf(fid,'%d\n',arm1CurrentLimit(i));
    fprintf(fid,'%d\n',arm2CurrentLimit(i));
    fprintf(fid,'%d\n',arm3CurrentLimit(i));
    fprintf(fid,'%d\n',arm4CurrentLimit(i));

    fprintf(fid,'%d\n',arm1TxRate(i));
    fprintf(fid,'%d\n',arm2TxRate(i));
    fprintf(fid,'%d\n',arm3TxRate(i));
    fprintf(fid,'%d\n',arm4TxRate(i));

    fprintf(fid,'%d\n',arm1KinInflCoeff(i));
    fprintf(fid,'%d\n',arm2KinInflCoeff(i));
    fprintf(fid,'%d\n',arm3KinInflCoeff(i));
    fprintf(fid,'%d\n',arm4KinInflCoeff(i));

    fprintf(fid,'%d\n',arm1FollowingError(i));
    fprintf(fid,'%d\n',arm2FollowingError(i));
    fprintf(fid,'%d\n',arm3FollowingError(i));
    fprintf(fid,'%d\n',arm4FollowingError(i));
end

fclose(fid);

% O programa está ON (mex function);
modoON()

keepgoing = 1;

q1 = [0.0 0.0 0.0 0.0 0.0 0.0]

drivebot(smart6)

firstPack = 0;

% Espera sinal verde;
while keepgoing == 1
    pause(1);
    j = esperaSinalVerde();
    
    if j == 1
        k = 0
        % Abre pacote de entrada
        A = load('saida.txt','r');
        
        n = 1;
        for i = 1:6
            SM(i) = A(n);
            n = n+1;
            D1(i) = A(n);
            n = n+1;
            D2(i) = A(n);
            n = n+1;
            D3(i) = A(n);
            n = n+1;
            D4(i) = A(n);
            n = n+1;
            D5(i) = A(n);
            n+1;
            EXT1(i) = A(n);
            n = n+1;
            EXT2(i) = A(n);
            n = n+1;
            EXT3(i) = A(n);
            n+1;
        end
        
        
        % Seta valores do pacote comum
        oSM = SM;
        oD1 = D1;
        oD2 = D2;
        oD3 = D3;
        oD4 = D4;
        oD5 = D5;
        oEXT1 = EXT1;
        oEXT2 = EXT2;
        oEXT3 = EXT3;
        
        % Escreve pacote Comum
        fid = fopen('entrada.txt','w');

        for i = 1:6
            fprintf(fid,'%f\n',oSM(i));
            fprintf(fid,'%f\n',oD1(i));
            fprintf(fid,'%f\n',oD2(i));
            fprintf(fid,'%f\n',oD3(i));
            fprintf(fid,'%f\n',oD4(i));
            fprintf(fid,'%f\n',oD5(i));
            fprintf(fid,'%f\n',oEXT1(i));
            fprintf(fid,'%f\n',oEXT2(i));
            fprintf(fid,'%f\n',oEXT3(i));
        end

        fclose(fid);

        % Efetua os movimentos definidos no pacote
        % ----------------------------------------
        for i = 1:6
            q1(i) = oD1(i);
        end
    
        fkine (smart6,q1);
        % ----------------------------------------

        abreSinal()
        
    elseif j == 0
        keepgoing = 0
        
    end
end
modoOFF()