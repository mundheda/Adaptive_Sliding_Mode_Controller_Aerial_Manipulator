% Compute torques, given current inputs, length, drag coefficient, and thrust coefficient.
function tau = torques(inputs, L, b, k)
 % Inputs are values for ?i
tau = [
L * k * (inputs(2) - inputs(4));
L * k * (inputs(3) - inputs(1));
b * (inputs(1) - inputs(2) + inputs(3) - inputs(4));
];
end
