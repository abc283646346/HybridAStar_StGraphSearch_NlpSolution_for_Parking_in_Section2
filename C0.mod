param Nfe;
param tf;
set I := {0..(Nfe-1)};
param VP{i in {1..8}};

var x{i in I};
var y{i in I};
var theta{i in I};
var v{i in I};
var a{i in I};
var phy{i in I};
var w{i in I};
var egoV{i in I, j in {1..4}, k in {1..2}};

minimize value:
1;
 
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

data;
param Nfe := include Nfe;
param tf := include TF;
param VP := include VP;