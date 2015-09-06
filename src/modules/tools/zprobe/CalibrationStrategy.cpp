// Generic calibration strategy based on nonlinear optimization
//
// Copyright (c) Oskar Linde 2015
//
// Licence: GPL v2

#include "CalibrationStrategy.h"
#include "Kernel.h"
#include "Config.h"
#include "Robot.h"
#include "StreamOutputPool.h"
#include "Gcode.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "PublicDataRequest.h"
#include "EndstopsPublicAccess.h"
#include "PublicData.h"
#include "Conveyor.h"
#include "ZProbe.h"
#include "BaseSolution.h"
#include "StepperMotor.h"
#include "Vector3.h"
#include <cstddef>
#include <fastmath.h>

#define radius_checksum       CHECKSUM("radius")
#define delta_calibration_strategy_checksum CHECKSUM("delta-calibration")

std::function<void(float[3])> compute_rotation_transform(float p, float q, float offset, float inverse_r[3]);
int endstop_parameter_index(char parameter);

bool CalibrationStrategy::update_parameter(char parameter, float delta) {
    return set_parameter(parameter, get_parameter(parameter) + delta);
}

void CalibrationStrategy::update_compensation_transformation()
{
    if (plane_u == 0 && plane_v == 0 && plane_offset == 0) {
        THEKERNEL->robot->compensationTransform = nullptr;
        inverse_r[0] = 0; inverse_r[1] = 0; inverse_r[2] = 1;
    } else
        THEKERNEL->robot->compensationTransform = compute_rotation_transform(plane_u, plane_v, plane_offset, inverse_r);
}

bool CalibrationStrategy::handleGcode(Gcode *gcode)
{
    if( gcode->has_g) {
        // G code processing
        if( gcode->g == 32 ) { // auto calibration for delta, Z bed mapping for cartesian
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_empty_queue();

            auto args = gcode->get_args();
            std::string parameters_to_optimize; parameters_to_optimize.reserve(args.size());
            int samples = 50;
            int repeats = 1;

            for (auto &v : args) {
                if (v.first == 'P')
                    samples = int(v.second);
                else if (v.first == 'O')
                    repeats = int(v.second);
                else
                    parameters_to_optimize += v.first;
            }

            // TODO: the following checks are delta specific
            if (parameters_to_optimize.empty()) {
                parameters_to_optimize = "XYZ"; // endstop only
            }
            // The W parameter is linearly dependent on X,Y,Z
            if (parameters_to_optimize.find('W') != std::string::npos &&
                    (parameters_to_optimize.find('X') != std::string::npos ||
                     parameters_to_optimize.find('Y') != std::string::npos ||
                     parameters_to_optimize.find('Z') != std::string::npos)) {
                gcode->stream->printf("Warning, parameter W is coupled to X,Y,Z. Optimize either W or XYZ. Ignoring W.\n");
                parameters_to_optimize.erase(parameters_to_optimize.find('W'));
            }

            // Make sure we initialize our trim variables
            get_parameter('X');
            get_parameter('Y');
            get_parameter('Z');

            // Make sure all parameters are valid
            for (auto p = parameters_to_optimize.begin(); p != parameters_to_optimize.end();) {
                // this is done by attempting to get and then set their values
                if (!update_parameter(*p,0)) {
                    gcode->stream->printf("Warning, invalid parameter '%c' ignored.\n", *p);
                    p = parameters_to_optimize.erase(p);
                } else {
                    p++;
                }
            }

            update_compensation_transformation(); // make sure no compensation transform from somewhere else is installed

            gcode->stream->printf("Commencing calibration of parameters: %s\n", parameters_to_optimize.c_str());

            samples = std::max(samples, (int)parameters_to_optimize.length());

            if (optimize_model(samples, repeats, parameters_to_optimize, gcode->stream)) {
                gcode->stream->printf("Calibration complete. Save settings with M500.\n");
            } else {
                gcode->stream->printf("Calibration may not have converged. Use M500 if you want to save settings anyway.\n");
            }

            // Check if endstop trim was updated
            if (parameters_to_optimize.find('X') != std::string::npos ||
                parameters_to_optimize.find('Y') != std::string::npos ||
                parameters_to_optimize.find('Z') != std::string::npos) {

                // Deficiency: homing is the only way to activate the endstop trim values
                zprobe->home();
            }

            return true;
        }
    } else if(gcode->has_m) {
        // handle mcodes
        if(gcode->m == 500 || gcode->m == 503) { // M500 save, M503 display
            gcode->stream->printf(";Plane tilt:\n");
            gcode->stream->printf("M567 U%.5f V%.5f W%.5f\n", plane_u, plane_v, plane_offset);
            return true;
        } else if(gcode->m == 567) {
            if (gcode->has_letter('U')) plane_u = gcode->get_value('U');
            if (gcode->has_letter('V')) plane_v = gcode->get_value('V');
            if (gcode->has_letter('W')) plane_offset = gcode->get_value('W');

            update_compensation_transformation();

            return true;
        }
    }

    return false;
}

bool CalibrationStrategy::handleConfig()
{
    // default is probably wrong
    float r= THEKERNEL->config->value(leveling_strategy_checksum, calibration_strategy_checksum, radius_checksum)->by_default(-1.0f)->as_number();
    if(r == -1) {
        r= THEKERNEL->config->value(leveling_strategy_checksum, delta_calibration_strategy_checksum, radius_checksum)->by_default(65.0f)->as_number();
    }
    this->probe_radius= r;
    return true;
}


bool CalibrationStrategy::setup_probe() {
    // home
    zprobe->home();
    zprobe->coordinated_move(NAN, NAN, 20, zprobe->getFastFeedrate(), false /* relative */);

    int s;
    if(!zprobe->run_probe(s, false)) return false;

    //float dive_height = zprobe->zsteps_to_mm(s) - zprobe->getProbeHeight(); // distance to move from home to 5mm above bed

    zprobe->coordinated_move(NAN, NAN, zprobe->getProbeHeight(), zprobe->getFastFeedrate(), true /* relative */);

    return true;
}


// fills out n actuator positions where the probe triggered
// returns false if any probe failed
bool CalibrationStrategy::probe_spiral(int n, int repeats, float actuator_positions[/*n*/][3]) {
    if (THEKERNEL->robot->actuators.size() != 3) return false;
    // Spiraling probe pattern

    // Archimedes' spiral r = a * theta
    // arc_length(x) = integrate a * theta * dtheta from 0 to x = a*x^2/2
    // inversely, theta(len) = sqrt(2*len/a)
    // we want outer point to be at probe_radius -> theta_max = probe_radius/a
    // step_length = length(theta_max)) / n == probe_radius^2/(2*a*n)
    // solve a * 2pi == step_length -> a = probe_radius / (2*sqrt(n)*sqrt(pi))

    float a = probe_radius / (2 * sqrt(n * M_PI));

    auto theta = [a](float length) {
        return sqrt(2*length/a);
    };
    double step_length = probe_radius * probe_radius / (2 * a * n);

    // move the probe to it's starting position a specified height above the bed
    setup_probe();

    for (int i = 0; i < n; i++) {
        float angle = theta(i * step_length);
        float r = angle * a;
        // polar to cartesian
        float x = r * cos(angle);
        float y = r * sin(angle);

        int steps; // dummy
        if (!zprobe->doProbeAt(steps, x, y, actuator_positions[i], repeats)) return false;
        // remove trim adjustment from returned actuator positions
        for (int j = 0; j < 3; j++)
            actuator_positions[i][j] += trim[j];
    }

    return true;
}

// fills out n actuator positions where the probe triggered
// returns false if any probe failed
bool CalibrationStrategy::probe_symmetric(int n, int repeats, float actuator_positions[/*n*/][3]) {
    if (THEKERNEL->robot->actuators.size() != 3) return false;

    // move the probe to it's starting position a specified height above the bed
    setup_probe();

    int position_index = 0;
    auto probe = [&](float x, float y) {
        int steps; // dummy
        bool success = zprobe->doProbeAt(steps, x, y, actuator_positions[position_index], repeats);
        // remove trim adjustment from returned actuator positions
        for (int j = 0; j < 3; j++)
            actuator_positions[position_index][j] += trim[j];
        position_index++;
        return success;
    };

    // probe the center point except for n==2 and n==3
    if (n != 2 && n != 3) {
        if (!probe(0,0)) return false;
        n--;
    }
    for (int i = 0; i < n; i++) {
        float angle = M_PI*2*i/n;

        float x = probe_radius * cos(angle);
        float y = probe_radius * sin(angle);
        if (!probe(x,y)) return false;
    }

    return true;
}


float CalibrationStrategy::compute_model_error(float const actuator_position[3], float home_offs_z) {
    float cartesian[3];
    float trimmed_position[3];
    // simulate the effect of trim
    for (int i = 0; i < 3; i++) trimmed_position[i] = actuator_position[i] - trim[i];
    THEKERNEL->robot->arm_solution->actuator_to_cartesian(trimmed_position, cartesian);

    // if home offset z is ≠ 0, we need to compensate for that
    cartesian[2] -= home_offs_z;

    // we want our model to map the plane to z == 0
    // Inverse compensationTransform for z
    double d = inverse_r[0] * cartesian[0] + inverse_r[1] * cartesian[1] + inverse_r[2] * cartesian[2];
    return d - inverse_r[2] * plane_offset;
}


float CalibrationStrategy::compute_model_rms_error(std::vector<V3> const& actuator_positions, float home_offs_z) {
    float err = 0;
    for (auto &actuator_position : actuator_positions) {
        float e = compute_model_error(actuator_position.m, home_offs_z);
        err += e*e;
    }
    return sqrt(err / actuator_positions.size());
}

void CalibrationStrategy::compute_JTJ_JTr(std::vector<V3> const& actuator_positions,
                                          std::string     const& parameters,
                                          std::vector<float>   & JTJ,
                                          std::vector<float>   & JTr,
                                          std::vector<float>   & scratch,
                                          float home_offs_z) {
    int m = parameters.size();
    scratch.resize(m);
    std::vector<float> &jacobian = scratch;

    // zero JTJ accumulator
    JTJ.resize(m * m);
    for (auto &v : JTJ) v = 0;

    JTr.resize(m);
    for (auto &v : JTr) v = 0;


    // iterating over residual terms rather than parameters first saves
    // a lot of memory at the cost of many more get/set_value calls
    for (int j = 0; j < (int)actuator_positions.size(); j++) {

        // compute numerically derivative of error w.r.t. parameters
        for (int i = 0; i < m; i++) {
            float x0 = get_parameter(parameters[i]);
            // step length for central difference approximation
            float h = std::max(fabsf(x0) * 1e-4, 1e-3);

            //compute central difference (f(x+h) - f(x-h)) / (2*h)
            set_parameter(parameters[i], x0 + h);
            jacobian[i] = compute_model_error(actuator_positions[j].m, home_offs_z);

            set_parameter(parameters[i], x0 - h);
            float one_over_2_h = 1./((x0+h)-(x0-h)); // trick to improve precision
            jacobian[i] = (jacobian[i] - compute_model_error(actuator_positions[j].m, home_offs_z)) * one_over_2_h;

            // restore the original value
            set_parameter(parameters[i], x0);
        }
        // Accumulate outer product of the Jacobian
        // Only compute upper triangle of the symmetric matrix
        for (int i = 0; i < m; i++) {
            for (int j = i; j < m; j++) {
                JTJ[i * m + j] += jacobian[i] * jacobian[j];
            }
        }

        float err = compute_model_error(actuator_positions[j].m, home_offs_z);
        for (int i = 0; i < m; i++) {
            JTr[i] -= jacobian[i] * err;
        }
    }

    // fill in the lower JTJ triangle
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < i; j++) {
            JTJ[i * m + j] = JTJ[j * m + i];
        }
    }
}

// Solve Ax=b for positive semidefinite A using LDL^T decomposition
// Destroys A and b in the process
// Result is stored in b
template<typename T>
void cholesky_backsub(int n, T A[/* n*n */], T b[/* n */]) {

    // Compute LDL^T decomposition
    for (int j = 0; j < n; j++) {
        T inv_diagonal = 1;
        for (int i = j; i < n; i++) {
            T v = A[i*n + j];
            for (int k = 0; k < j; k++) {
                v -= A[k*n + j] * A[i*n + k];
            }
            if (i == j) {
                A[i*n + j] = v;
                inv_diagonal = 1/v;
            } else {
                A[j*n + i] = v; // store undivided value
                A[i*n + j] = v * inv_diagonal;
            }
        }
    }

    // Compute A^-1*b, A=LDL^T
    // L
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < i; j++) {
            b[i] -= A[i*n + j] * b[j];
        }
    }

    // D
    for (int i = 0; i < n; i++) {
        b[i] /= A[i*n + i];
    }

    // L^T
    for (int i = n-1; i >= 0; i--) {
        for (int j = i+1; j < n; j++) {
            b[i] -= A[j*n + i] * b[j];
        }
    }
}

bool CalibrationStrategy::probe_pattern(int n, int repeats, float actuator_positions[/*N*/][3]) {
    if (n <= 7) {
        return probe_symmetric(n, repeats, &actuator_positions[0]);
    } else {
        return probe_spiral(n, repeats, &actuator_positions[0]);
    }
}

bool CalibrationStrategy::optimize_model(int n, int repeats, std::string const& parameters, StreamOutput* stream) {

    std::vector<V3> actuator_positions(n);
    std::vector<float> scratch;
    std::vector<float> JTJ;
    std::vector<float> JTr;
    stream->printf("Probing %d points (%d repeat(s) per point).\n", n, repeats);

    if (!probe_pattern(n, repeats, &actuator_positions[0].m)) {
        stream->printf("ERROR: Probing failed!\n");
        return false;
    }

//    for (auto &v : actuator_positions) {
//        stream->printf("%.5f %.5f %.5f\n",v.m[0],v.m[1],v.m[2]);
//    }
    float home_offs[3] = {};

    {
        void *returned_data;
        if (PublicData::get_value(endstops_checksum, home_offset_checksum, &returned_data)) {
            for (int i = 0; i < 3; i++) home_offs[i] = static_cast<float *>(returned_data)[i];
        }
    }

    int m = parameters.size();

    // Save initial values so we can roll back if we end up with a worse solution
    std::vector<float> stored_values(m);
    auto save = [&]() {
        for (int i = 0; i < m; i++)
            stored_values[i] = get_parameter(parameters[i]);
    };
    auto restore = [&]() {
        for (int i = 0; i < m; i++)
            set_parameter(parameters[i], stored_values[i]);
    };
    auto print_state = [&]() {
        for (int i = 0; i < m; i++)
            stream->printf("%c:%.3g ", parameters[i], stored_values[i]);
        stream->printf("\n");
    };

    save();

    float last_error = compute_model_rms_error(actuator_positions, home_offs[2]);
    print_state();
    stream->printf("RMS error before optimization: %.3f mm\n", last_error);

    static const int max_iterations = 50; // TODO: config parameter
    std::vector<float> A;
    std::vector<float> b;

    // Lambda is a damping factor. Higher lambda makes the optimization
    // more robust to bad initial estiamtes, but also makes it converge slower
    float lambda = 1; // TODO: config parameter?
    for (int iteration = 0; iteration < max_iterations; iteration++) {
        compute_JTJ_JTr(actuator_positions, parameters, JTJ, JTr, scratch, home_offs[2]);

again:
        A = JTJ;
        b = JTr;
        // Damp the problem a bit
        for (int i = 0; i < m; i++) {
            A[i*m + i] *= (1 + lambda);
        }
        cholesky_backsub(m, A.data(), b.data());
        stream->printf("Attempting delta:");
        for (auto v : b)
            stream->printf("%f ",v);
        stream->printf("\n");
        for (int i = 0; i < m; i++) {
            update_parameter(parameters[i], b[i]);
        }

        float new_error = compute_model_rms_error(actuator_positions, home_offs[2]);
        stream->printf("RMS error after iteration %d: %.3f mm", iteration+1, new_error);

        float sq_sum_delta = 0;
        for (auto d : b) sq_sum_delta += d*d;

        if (!(new_error < last_error)) {
            restore();
            if (sq_sum_delta < 0.0001 * 0.0001) {
                stream->printf("\nConverged\n");
                return true;
            }

            stream->printf(" -> rolling back\n");
            // increase damping
            lambda *= 3;
            if (lambda > 1e10) {
                stream->printf("Max lambda\n");
                return false;
            }
            goto again; // adjusted lambda, no need to recompute JTJ, JTr
        } else {
            float improvement = (1 - new_error / last_error);
            stream->printf(" (%.3f%% improvement)\n", 100*improvement);
            last_error = new_error;
            save();
            print_state();

            if (improvement < 0.0001 ||
                sq_sum_delta < 0.0001 * 0.0001) {
                // improvement < 0.01 % or < 0.1 µm adjustment
                stream->printf("Done\n");
                return true;
            }

            // reduce damping
            lambda /= 2;
            if (lambda < 1e-7) lambda = 1e-7; // Reasonable lower limit for float precision
        }
    }
    if (last_error < 0.001) return true; // We are within 1 µm, let's call that a success

    stream->printf("Failed to converge after %d iterations\n", max_iterations);
    return false;
}


int endstop_parameter_index(char parameter) {
    switch(parameter) {
    case 'X': return 0;
    case 'Y': return 1;
    case 'Z': return 2;
    default: return -1;
    }
}

std::function<void(float[3])> compute_rotation_transform(float p, float q, float offset, float inverse_r[3]) {
    // setup rotation matrix
    Vector3 x(1,0,0),
            y(0,1,0),
            z(p,q,sqrt(1-p*p-q*q));
    // orthogonalize
    y = z.cross(x).unit();
    x = y.cross(z); // y and z are now orthogonal unit vectors

    // store last row of R
    inverse_r[0] = x[2];
    inverse_r[1] = y[2];
    inverse_r[2] = z[2];

    return [x,y,z,offset](float p[3]) {
        Vector3 q(p[0],p[1],p[2]);
        // Note: multiply by R^T
        p[0] = q.dot(x);
        p[1] = q.dot(y);
        p[2] = q.dot(z) + offset;
    };
}


bool CalibrationStrategy::set_parameter(char parameter, float value) {
    if (parameter == 'U' || parameter == 'V' || parameter == 'W') {
        if (parameter == 'U') plane_u = value;
        if (parameter == 'V') plane_v = value;
        if (parameter == 'W') plane_offset = value;

        update_compensation_transformation();
        return true;
    }
    int endstop = endstop_parameter_index(parameter);
    if (endstop >= 0) {
        void *returned_data;

        if (PublicData::get_value(endstops_checksum, trim_checksum, &returned_data)) {
            // refresh cache
            float *rtrim = static_cast<float *>(returned_data);
            trim[0] = rtrim[0]; trim[1] = rtrim[1]; trim[2] = rtrim[2];

            trim[endstop] = value;
            return PublicData::set_value( endstops_checksum, trim_checksum, trim);
        }
        return false;
    } else {
        BaseSolution::arm_options_t options;
        if(THEKERNEL->robot->arm_solution->get_optional(options)) {
            options[parameter] = value;
            return THEKERNEL->robot->arm_solution->set_optional(options);
        }
        return false;
    }
}


float CalibrationStrategy::get_parameter(char parameter) {
    if (parameter == 'U') return plane_u;
    if (parameter == 'V') return plane_v;
    if (parameter == 'W') return plane_offset;

    int endstop = endstop_parameter_index(parameter);
    if (endstop >= 0) {
        void *returned_data;
        bool ok = PublicData::get_value(endstops_checksum, trim_checksum, &returned_data);

        if (ok) {
            float *rtrim = static_cast<float *>(returned_data);
            trim[endstop] = rtrim[endstop];
            return rtrim[endstop];
        }
    } else {
        BaseSolution::arm_options_t options;
        if(THEKERNEL->robot->arm_solution->get_optional(options)) {
            return options[parameter];
        }
    }
    return NAN;
}
