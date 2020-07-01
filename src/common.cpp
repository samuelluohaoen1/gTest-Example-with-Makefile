#include "common.hpp"

using namespace arma;


/* Note: the functions here generally don't check the validity 
 *       of the shape of the vectors/matrices parameters.
 *       (design by contract)
 */



arma::vec unit_vec_x = {1, 0};
arma::vec unit_vec_y = {0, 1};


arma::mat rotation_matrix_2D(double angle_degree) {
    double x = to_radian(angle_degree);
    mat rot = {{cos(x), -sin(x)},
               {sin(x),  cos(x)}};
    return rot;
}



// from standard basis [(1,0), (0,1)] to basis [\vec{vx}, \vec{vy}]relative to std basis
arma::mat change_basis_matrix_2D(arma::vec vx, arma::vec vy) {
    mat P_inv = {{vx(0), vy(0)},
                 {vx(1), vy(1)}};
    return inv(P_inv);
}

/* from basis [\vec{vax}, \vec{vay}] to basis [\vec{vbx}, \vec{vby}] 
 * (both basis sets' vectors are defined under the standard basis) */
//arma::mat change_basis_matrix_2D(arma::vec vax, arma::vec vay, 
//                                 arma::vec vbx, arma::vec vby) {}