/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    Arm.cpp

    @version    0.1
    @author     Miika 'LehdaRi' Lehtimäki
    @date       2015-04-24

**/


#include "Arm.hpp"

#include <iostream>


Arm::Arm(Shader& shader, std::vector<Matrix4Glf> matrices) {
    for (auto& m : matrices)
        joints_.emplace_back(shader, m);
}

template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon =
std::numeric_limits<double>::epsilon())
{
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU |
    Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows())
    *svd.singularValues().array().abs()(0);
        return svd.matrixV() *  (svd.singularValues().array().abs() >
    tolerance).select(svd.singularValues().array().inverse(),
    0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

void Arm::solve(Vector3Glf goal_point, int life_count) {
    // prev and curr are for use of halving
    // last is making sure the iteration gets a better solution than the last iteration,
    // otherwise revert changes
    float prev_err, curr_err, last_err = 9999;
    Vector3Glf current_point;
    int max_iterations = 200;
    int count = 0;
    float err_margin = 0.01;

    //goal_point -= base;
    if (goal_point.norm() > get_max_length()) {
        goal_point = goal_point.normalized() * get_max_length();
    }

    current_point = calculate_end_effector();

    // save the first err
    prev_err = (goal_point - current_point).norm();
    curr_err = prev_err;
    last_err = curr_err;

    // while the current point is close enough, stop iterating
    while (curr_err > err_margin) {
        // calculate the difference between the goal_point and current_point
        Vector3f dP = goal_point - current_point;

        // create the jacovian
        int segment_size = joints_.size();

        // build the transpose matrix (easier for eigen matrix construction)
        Eigen::MatrixXf jac_t(3*segment_size, 3);
        for(int i=0; i<segment_size; ++i) {
            Eigen::Matrix<float, 1, 3> row_theta = compute_jacovian_segment(i, goal_point, joints_[i].getRight());
            Eigen::Matrix<float, 1, 3> row_phi = compute_jacovian_segment(i, goal_point, joints_[i].getUp());
            Eigen::Matrix<float, 1, 3> row_z = compute_jacovian_segment(i, goal_point, joints_[i].getForward());

            int j = 3*i;
            jac_t(j, 0) = row_theta(0, 0);
            jac_t(j, 1) = row_theta(0, 1);
            jac_t(j, 2) = row_theta(0, 2);

            jac_t(j+1, 0) = row_phi(0, 0);
            jac_t(j+1, 1) = row_phi(0, 1);
            jac_t(j+1, 2) = row_phi(0, 2);

            jac_t(j+2, 0) = row_z(0, 0);
            jac_t(j+2, 1) = row_z(0, 1);
            jac_t(j+2, 2) = row_z(0, 2);
        }
        // compute the final jacovian
        Eigen::MatrixXf jac(3, 3*segment_size);
        jac = jac_t.transpose();

        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> pseudo_ijac;
        Eigen::MatrixXf pinv_jac(3*segment_size, 3);
        pinv_jac = pseudoInverse(jac);

        Eigen::Matrix<float, Eigen::Dynamic, 1> changes = pinv_jac * dP;

        std::cout << "changes: " << changes << std::endl;

        auto undoJoints = joints_;
        auto lastJoints = joints_;

        for(int i=0; i<segment_size; ++i) {
            // save the current transformation on the segments
            //joints_[i].save_transformation();

            int j = i*3;
            // apply the change to the theta angle
            //joints_[i].apply_angle_change(changes[j], joints_[i].get_right());
            // apply the change to the phi angle
            //joints_[i].apply_angle_change(changes[j+1], joints_[i].get_up());
            // apply the change to the z angle
            joints_[i].setTheta(joints_[i].getTheta() + changes[j+2]);
        }

        // compute current_point after making changes
        current_point = calculate_end_effector();

        //cout << "current_point: " << vectorString(current_point) << endl;
        //cout << "goal_point: " << vectorString(goal_point) << endl;

        prev_err = curr_err;
        curr_err = (goal_point - current_point).norm();

        int halving_count = 0;

        std::cout << "curr err: " << curr_err << " || prev err: " << prev_err << " || last err: " << last_err << std::endl;
        // make sure we aren't iterating past the solution
        while (curr_err > last_err) {
            // undo changes
            //for(int i=0; i<segment_size; i++) {
                // unapply the change to the saved angle
                //joints_[i].load_transformation();
            //}

            joints_ = lastJoints;
            current_point = calculate_end_effector();
            changes *= 0.5;
            // reapply halved changes

            lastJoints = joints_;

            for(int i=0; i<segment_size; ++i) {
                // save the current transformation on the segments
                //joints_[i].save_transformation();

                int j = 3*i;
                // apply the change to the theta angle
                //joints_[i].apply_angle_change(changes[j], joints_[i].getRight());
                // apply the change to the phi angle
                //joints_[i].apply_angle_change(changes[j+1], joints_[i].getUp());
                // apply the change to the z angle
                joints_[i].setTheta(joints_[i].getTheta() + changes[j+2]);
            }

            // compute the end_effector and measure error
            current_point = calculate_end_effector();
            prev_err = curr_err;
            curr_err = (goal_point - current_point).norm();

            std::cout << "|half| curr err: " << curr_err << " || prev err: " << prev_err << std::endl;
            halving_count++;
            if (halving_count > 100)
                break;
        }

        if (curr_err > last_err) {
            // undo changes
            //for(int i=0; i<segment_size; i++) {
                // unapply the change to the saved angle
                //segments[i]->load_last_transformation();
            //}

            joints_ = undoJoints;

            current_point = calculate_end_effector();
            curr_err = (goal_point - current_point).norm();
            std::cout << "curr iteration not better than last, reverting" << std::endl;
            std::cout << "curr err: " << curr_err << " || last err: " << last_err << std::endl;
            break;
        }
        /*for(int i=0; i<segment_size; i++) {
            // unapply the change to the saved angle
            joints_[i].save_last_transformation();
        }*/
        std::cout << "curr err: " << curr_err << " || last err: " << last_err << std::endl;
        last_err = curr_err;
        std::cout << "last_err is now : " << last_err << std::endl;


        // make sure we don't infinite loop
        count++;
        if (count > max_iterations) {
            break;
        }
    }

    /*
    // if we haven't gotten to a nice solution
    if (curr_err > err_margin) {
        // kill off infinitely recursive solutions
        if (life_count <= 0) {
            return;
        }
        // try to solve it again
        solve(goal_point, life_count-1);
    } else {
    */
    std::cout << "final error: " << curr_err << std::endl;
}

Eigen::Matrix<float, 1, 3> Arm::compute_jacovian_segment(int seg_num, Vector3Glf goal_point, Vector3f angle) {
    Joint& j = joints_.at(seg_num);
    // mini is the amount of angle you go in the direction for numerical calculation
    float mini = 0.0005;

    Vector3Glf transformed_goal = goal_point;
    for(int i=joints_.size()-1; i>seg_num; i--) {
        // transform the goal point to relevence to this segment
        // by removing all the transformations the segments afterwards
        // apply on the current segment
        transformed_goal -= joints_[i].getEndPoint();
    }

    Vector3Glf my_end_effector = calculate_end_effector(seg_num);

    // transform them both to the origin
    if (seg_num-1 >= 0) {
        my_end_effector -= calculate_end_effector(seg_num-1);
        transformed_goal -= calculate_end_effector(seg_num-1);
    }

    // original end_effector
    Vector3Glf original_ee = calculate_end_effector();

    // angle input is the one you rotate around
    // remove all the rotations from the previous segments by applying them
    Eigen::AngleAxis<GLfloat> t = Eigen::AngleAxis<GLfloat>(mini, angle);

    Matrix4Glf m, mi;

    m << t.matrix(),        Vector3Glf(0.0f, 0.0f, 0.0f),
         0.0f, 0.0f, 0.0f,  1.0f;

    mi <<   t.inverse().matrix(),   Vector3Glf(0.0f, 0.0f, 0.0f),
            0.0f, 0.0f, 0.0f,       1.0f;

    // transform the segment by some delta(theta)
    j.setJointMatrix(m * j.getJointMatrix());
    // new end_effector
    Vector3Glf new_ee = calculate_end_effector();
    // reverse the transformation afterwards
    j.setJointMatrix(mi * j.getJointMatrix());

    // difference between the end_effectors
    // since mini is very small, it's an approximation of
    // the derivative when divided by mini
    Vector3f diff = new_ee - original_ee;

    // return the row of dx/dtheta, dy/dtheta, dz/dtheta
    Eigen::Matrix<float, 1, 3> ret;
    ret << diff[0]/mini, diff[1]/mini, diff[2]/mini;
    return ret;
}

// computes end_effector up to certain number of segments
Vector3Glf Arm::calculate_end_effector(int segment_num /* = -1 */) {
    int segment_num_to_calc = segment_num;

    // if default value, compute total end effector
    if (segment_num == -1) {
        segment_num_to_calc = joints_.size() - 1;
    }
    // else don't mess with it

    // start with base
    Vector3Glf ret(0.0f, 0.0f, 0.0f);
    for(int i=0; i<=segment_num_to_calc; i++) {
        // add each segments end point vector to the base
        ret += joints_[i].getEndPoint();
    }
    // return calculated end effector
    return ret;
}

float Arm::get_max_length(void) {
    float ret = 0;
    for(unsigned int i=0; i<joints_.size(); i++) {
        ret += joints_[i].getLength();
    }
    return ret;
}
