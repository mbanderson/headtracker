% AA 273 | Final Project | Quaternion to DCM
% Jin Woo Park
% June 1, 2017

function DCM = getRw2b(q)

% Computes the Directional Cosine Matrix (Rotation Matrix) for given
% quaternion.
%
% INPUTS:
%       q: quaternion at time t
%
% OUTPUTS:
%     DCM: Directional Cosine Matrix (Rotation Matrix).

    qx = q(1);
    qy = q(2);
    qz = q(3);
    qscal = q(4);
    qvec = [qx; qy; qz];
    qcross = [0 -qz qy; qz 0 -qx; -qy qx 0];
    DCM = (qscal^2 - norm(qvec)^2)*eye(3,3) + 2*qvec*qvec' - 2*qscal*qcross;
end