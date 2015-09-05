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

int endstop_parameter_index(char parameter);

bool CalibrationStrategy::update_parameter(char parameter, float delta) {
    return set_parameter(parameter, get_parameter(parameter) + delta);
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
                if (v.first == 'N')
                    samples = int(v.second);
                else if (v.first == 'O')
                    repeats = int(v.second);
                else
                    parameters_to_optimize += v.first;
            }

            if (parameters_to_optimize.empty()) {
                parameters_to_optimize = "XYZ"; // endstop only
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

            gcode->stream->printf("Commencing calibration of parameters: %s\n", parameters_to_optimize.c_str());

            samples = std::max(samples, (int)parameters_to_optimize.length());

            if (optimize_delta_model(samples, repeats, parameters_to_optimize, gcode->stream)) {
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

struct V3 { float m[3]; };

float CalibrationStrategy::compute_model_error(float const actuator_position[3]) {
    float cartesian[3];
    float trimmed_position[3];
    // simulate the effect of trim
    for (int i = 0; i < 3; i++) trimmed_position[i] = actuator_position[i] - trim[i];
    THEKERNEL->robot->arm_solution->actuator_to_cartesian(trimmed_position, cartesian);
    // we want our model to map the plane to z == 0
    return cartesian[2];
}

float CalibrationStrategy::compute_model_rms_error(std::vector<V3> const& actuator_positions) {
    float err = 0;
    for (auto &actuator_position : actuator_positions) {
        float e = compute_model_error(actuator_position.m);
        err += e*e;
    }
    return sqrt(err / actuator_positions.size());
}

void CalibrationStrategy::compute_JTJ_JTr(std::vector<V3> const& actuator_positions,
                     std::string     const& parameters,
                     std::vector<float>   & JTJ,
                     std::vector<float>   & JTr,
                     std::vector<float>   & scratch) {
    int m = parameters.size();
    scratch.resize(m);
    std::vector<float> &jacobian = scratch;

    // zero JTJ accumulator
    JTJ.resize(m * m);
    for (auto &v : JTJ) v = 0;

    JTr.resize(m);
    for (auto &v : JTr) v = 0;

    // step length for central difference approximation
    float h = 1e-4;
    float one_over_2_h = 1. / (2 * h);

    // iterating over residual terms rather than parameters first saves
    // a lot of memory at the cost of much more get/set_value calls
    for (int j = 0; j < (int)actuator_positions.size(); j++) {

        // compute numerically derivative of error w.r.t. parameters
        for (int i = 0; i < m; i++) {
            float x0 = get_parameter(parameters[i]);

            //compute central difference (f(x+h) - f(x-h)) / (2*h)
            set_parameter(parameters[i], x0 + h);
            jacobian[i] = compute_model_error(actuator_positions[j].m);

            set_parameter(parameters[i], x0 - h);
            jacobian[i] = (jacobian[i] - compute_model_error(actuator_positions[j].m)) * one_over_2_h;

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

        float err = compute_model_error(actuator_positions[j].m);
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

bool CalibrationStrategy::optimize_delta_model(int n, int repeats, std::string const& parameters, StreamOutput* stream) {

    std::vector<V3> actuator_positions(n);
    std::vector<float> scratch;
    std::vector<float> JTJ;
    std::vector<float> JTr;
    stream->printf("Probing %d points (%d repeat(s) per point).\n", n, repeats);
    if (!probe_spiral(n, repeats, &actuator_positions[0].m)) return false;

    for (auto &v : actuator_positions) {
        stream->printf("%.5f %.5f %.5f\n",v.m[0],v.m[1],v.m[2]);
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
            stream->printf("%c:%.2f ", parameters[i], stored_values[i]);
        stream->printf("\n");
    };

    save();

    float last_error = compute_model_rms_error(actuator_positions);
    print_state();
    stream->printf("RMS error before optimization: %.3f mm\n", last_error);

    static const int max_iterations = 50;
    std::vector<float> A;
    std::vector<float> b;

    float lambda = 1;
    for (int iteration = 0; iteration < max_iterations; iteration++) {
        compute_JTJ_JTr(actuator_positions, parameters, JTJ, JTr, scratch);

again:
        A = JTJ;
        b = JTr;
        // Damp the problem a bit
        for (int i = 0; i < m; i++) {
            A[i*m + i] *= (1 + lambda);
        }
        cholesky_backsub(m, A.data(), b.data());
        stream->printf("delta:");
        for (auto v : b)
            stream->printf("%f ",v);
        stream->printf("\n");
        for (int i = 0; i < m; i++) {
            update_parameter(parameters[i], b[i]);
        }

        float new_error = compute_model_rms_error(actuator_positions);
        print_state();
        stream->printf("RMS error after iteration %d: %.3f mm", iteration+1, new_error);
        if (new_error > last_error) {
            stream->printf(" -> rolling back\n");
            restore();
            // increase damping
            lambda *= 3;
            if (lambda > 1e10) {
                stream->printf("Max lambda\n");
                return false;
            }
            goto again;
        } else {
            float improvement = (1 - new_error / last_error);
            stream->printf(" (%.3f%% improvement)\n", 100*improvement);
            last_error = new_error;
            save();

            if (improvement < 0.0001) {
                stream->printf("Done\n");
                print_state();
                return true;
            }

            // reduce damping
            lambda /= 2;
            if (lambda < 1e-7) lambda = 1e-7;
        }
    }
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



bool CalibrationStrategy::set_parameter(char parameter, float value) {
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
