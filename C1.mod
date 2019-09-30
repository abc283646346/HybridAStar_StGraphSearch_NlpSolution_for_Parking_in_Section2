param Nfe;
param tf;
param NE = Nfe - 1;
param Nobs;
param hi = tf / NE;
set I := {0..NE};
set I1 := {1..NE};
param OV{i in I, j in {1..Nobs}, k in {1..4}, n in {1..2}};
param Area{i in {1..Nobs}};
param BV_config{i in {1..14}};
param VP{i in {1..8}};
param AreaVehicle = (VP[1] + VP[2]) * 2 * VP[3];

var x{i in I};
var y{i in I};
var theta{i in I};
var v{i in I};
var a{i in I};
var phy{i in I};
var w{i in I};
var egoV{i in I, j in {1..4}, k in {1..2}};

minimize cost_function:
sum{i in I}(w[i]^2 + a[i]^2) + sum{i in I}(v[i]^2 + phy[i]^2);

### ODEs ###
s.t. DIFF_dxdt {i in I1}:
x[i] = x[i-1] + hi * v[i] * cos(theta[i]);
s.t. DIFF_dydt {i in I1}:
y[i] = y[i-1] + hi * v[i] * sin(theta[i]);
s.t. DIFF_dvdt {i in I1}:
v[i] = v[i-1] + hi * a[i];
s.t. DIFF_dthetadt {i in I1}:
theta[i] = theta[i-1] + hi * tan(phy[i]) * v[i] * VP[4];
s.t. DIFF_dphydt {i in I1}:
phy[i] = phy[i-1] + hi * w[i];

### Equations for vertexes A B C D ###
s.t. RELATIONSHIP_AX {i in I}:
egoV[i,1,1] = x[i] + VP[1] * cos(theta[i]) - VP[3] * sin(theta[i]);
s.t. RELATIONSHIP_BX {i in I}:
egoV[i,2,1] = x[i] + VP[1] * cos(theta[i]) + VP[3] * sin(theta[i]);
s.t. RELATIONSHIP_CX {i in I}:
egoV[i,3,1] = x[i] - VP[2] * cos(theta[i]) + VP[3] * sin(theta[i]);
s.t. RELATIONSHIP_DX {i in I}:
egoV[i,4,1] = x[i] - VP[2] * cos(theta[i]) - VP[3] * sin(theta[i]);
s.t. RELATIONSHIP_AY {i in I}:
egoV[i,1,2] = y[i] + VP[1] * sin(theta[i]) + VP[3] * cos(theta[i]);
s.t. RELATIONSHIP_BY {i in I}:
egoV[i,2,2] = y[i] + VP[1] * sin(theta[i]) - VP[3] * cos(theta[i]);
s.t. RELATIONSHIP_CY {i in I}:
egoV[i,3,2] = y[i] - VP[2] * sin(theta[i]) - VP[3] * cos(theta[i]);
s.t. RELATIONSHIP_DY {i in I}:
egoV[i,4,2] = y[i] - VP[2] * sin(theta[i]) + VP[3] * cos(theta[i]);

### Manifold constraints w.r.t. vehicle physics ###
s.t. Bonds_phy {i in I}:
-VP[7] <= phy[i] <= VP[7];
s.t. Bonds_a {i in I}:
-VP[6] <= a[i] <= VP[6];
s.t. Bonds_v {i in I}:
-VP[5] <= v[i] <= VP[5];
s.t. Bonds_w {i in I}:
-VP[8] <= w[i] <= VP[8];

### Collision avoidance 1 ###
s.t. ObsVertexOutOfABCD {i in I, j in {1..Nobs}, k in {1..4}}:
0.5 * abs(OV[i,j,k,1] * egoV[i,1,2] + egoV[i,1,1] * egoV[i,2,2] + egoV[i,2,1] * OV[i,j,k,2] - OV[i,j,k,1] * egoV[i,2,2] - egoV[i,1,1] * OV[i,j,k,2] - egoV[i,2,1] * egoV[i,1,2]) + 
0.5 * abs(OV[i,j,k,1] * egoV[i,3,2] + egoV[i,3,1] * egoV[i,2,2] + egoV[i,2,1] * OV[i,j,k,2] - OV[i,j,k,1] * egoV[i,2,2] - egoV[i,3,1] * OV[i,j,k,2] - egoV[i,2,1] * egoV[i,3,2]) + 
0.5 * abs(OV[i,j,k,1] * egoV[i,3,2] + egoV[i,3,1] * egoV[i,4,2] + egoV[i,4,1] * OV[i,j,k,2] - OV[i,j,k,1] * egoV[i,4,2] - egoV[i,3,1] * OV[i,j,k,2] - egoV[i,4,1] * egoV[i,3,2]) + 
0.5 * abs(OV[i,j,k,1] * egoV[i,1,2] + egoV[i,1,1] * egoV[i,4,2] + egoV[i,4,1] * OV[i,j,k,2] - OV[i,j,k,1] * egoV[i,4,2] - egoV[i,1,1] * OV[i,j,k,2] - egoV[i,4,1] * egoV[i,1,2]) >= AreaVehicle + 0.2;

s.t. CarVertexOutOfObstacle {i in I, j in {1..Nobs}, k in {1..4}}:
0.5 * abs(egoV[i,k,1] * OV[i,j,1,2] + OV[i,j,1,1] * OV[i,j,2,2] + OV[i,j,2,1] * egoV[i,k,2] - egoV[i,k,1] * OV[i,j,2,2] - OV[i,j,1,1] * egoV[i,k,2] - OV[i,j,2,1] * OV[i,j,1,2]) + 
0.5 * abs(egoV[i,k,1] * OV[i,j,3,2] + OV[i,j,3,1] * OV[i,j,2,2] + OV[i,j,2,1] * egoV[i,k,2] - egoV[i,k,1] * OV[i,j,2,2] - OV[i,j,3,1] * egoV[i,k,2] - OV[i,j,2,1] * OV[i,j,3,2]) + 
0.5 * abs(egoV[i,k,1] * OV[i,j,3,2] + OV[i,j,3,1] * OV[i,j,4,2] + OV[i,j,4,1] * egoV[i,k,2] - egoV[i,k,1] * OV[i,j,4,2] - OV[i,j,3,1] * egoV[i,k,2] - OV[i,j,4,1] * OV[i,j,3,2]) + 
0.5 * abs(egoV[i,k,1] * OV[i,j,1,2] + OV[i,j,1,1] * OV[i,j,4,2] + OV[i,j,4,1] * egoV[i,k,2] - egoV[i,k,1] * OV[i,j,4,2] - OV[i,j,1,1] * egoV[i,k,2] - OV[i,j,4,1] * OV[i,j,1,2]) >= Area[j] + 0.2;

### Collision avoidance 2 ###
s.t. CarVertexInBox {i in I, j in {1..4}, k in {1..2}}:
-20 <= egoV[i,j,k] <= 20;

############# Two-Point Boundary Values #############
s.t. EQ_init_x :
x[0] = BV_config[1];
s.t. EQ_init_y :
y[0] = BV_config[2];
s.t. EQ_init_theta :
theta[0] = BV_config[3];
s.t. EQ_init_v :
v[0] = BV_config[4];
s.t. EQ_init_a :
a[0] = BV_config[5];
s.t. EQ_init_phy :
phy[0] = BV_config[6];
s.t. EQ_init_w :
w[0] = BV_config[7];

s.t. EQ_end_x :
x[NE] = BV_config[8];
s.t. EQ_end_y :
y[NE] = BV_config[9];
s.t. EQ_end_theta_sin :
sin(theta[NE]) = sin(BV_config[10]);
s.t. EQ_end_theta_cos :
cos(theta[NE]) = cos(BV_config[10]);
s.t. EQ_end_v :
v[NE] = BV_config[11];
s.t. EQ_end_a :
a[NE] = BV_config[12];
s.t. EQ_end_phy :
phy[NE] = BV_config[13];
s.t. EQ_end_w :
w[NE] = BV_config[14];

data;
param Nfe := include Nfe;
param tf := include TF;
param Nobs := include Nobs;
param OV := include OV;
param Area := include Area;
param BV_config := include BV_config;
param VP := include VP;