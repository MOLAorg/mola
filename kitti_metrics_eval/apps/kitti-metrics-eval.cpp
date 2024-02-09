/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   kitti-metrics-eval
 * @brief  Evaluate error of Kitti dataset resulting path vs. Ground Truth
 * @author Jose Luis Blanco Claraco
 * @date   Jan 25, 2019
 *
 * This file is largely based on the public Kitti dataset evaluation C++ code.
 * Rewritten to use Eigen instead of GNU GPL'd code.
 *
 * ###########################################################################
 * #   THE KITTI VISION BENCHMARK SUITE: VISUAL ODOMETRY / SLAM BENCHMARK    #
 * #              Andreas Geiger    Philip Lenz    Raquel Urtasun            #
 * #                    Karlsruhe Institute of Technology                    #
 * #                Toyota Technological Institute at Chicago                #
 * #                             www.cvlibs.net                              #
 * ###########################################################################
 */

#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>  // tokenize()

#include <Eigen/Dense>
#include <array>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <optional>

// Declare supported cli switches ===========
static TCLAP::CmdLine cmd("kitti-metrics-eval");

static TCLAP::ValueArg<std::string> arg_kitti_basedir(
    "k", "kitti-basedir",
    "Path to the kitti datasets. Overrides to the default, which is reading "
    "the env var `KITTI_BASE_DIR`.",
    false, "", "", cmd);

static TCLAP::ValueArg<std::string> arg_result_path(
    "r", "result-tum-path", "File to evaluate, in TUM format", true,
    "result.txt|result_%02i.txt", "result.txt", cmd);

static TCLAP::ValueArg<std::string> argSavePathKittiFormat(
    "", "save-as-kitti",
    "If given, will transform the input path from the LIDAR frame to the cam0 "
    "frame and save the path to a TXT file in the format expected by KITTI dev "
    "kit.",
    false, "result.kitti", "result.kitti", cmd);

static TCLAP::MultiArg<int> arg_seq(
    "s", "sequence",
    "The sequence number of the path(s) file(s) to evaluate, used to find out "
    "GT and calibration files for the Kitti dataset.",
    false, "01", cmd);

static TCLAP::ValueArg<std::string> arg_override_gt_file(
    "", "gt-tum-path",
    "If provided, the --sequence flag will be ignored and this particular file "
    "in TUM format will be read and used as ground truth to compare against "
    "the resulting odometry path.",
    false, "trajectory_gt.txt", "trajectory_gt.txt", cmd);

static TCLAP::SwitchArg argSkipFigures(
    "", "no-figures", "Skip generating the error figures", cmd);

static std::string kitti_basedir;
// points to CPose3D path from odometry/slam

static bool eval();

static void do_kitti_eval_error()
{
    using namespace std::string_literals;

    if (const char* s = ::getenv("KITTI_BASE_DIR"); s != nullptr)
        kitti_basedir = s;

    if (kitti_basedir.empty())
    {
        if (arg_kitti_basedir.isSet())
            kitti_basedir = arg_kitti_basedir.getValue();
        else
        {
            throw std::runtime_error(
                "Error: Either set the env var `KITTI_BASE_DIR` or the command "
                "line argument --kitti-basedir");
        }
    }
    ASSERT_DIRECTORY_EXISTS_(kitti_basedir);
    std::cout << "Using kitti datasets basedir: " << kitti_basedir << "\n";

    if (!arg_override_gt_file.isSet())
        ASSERT_DIRECTORY_EXISTS_(kitti_basedir + "/poses"s);

    // Run evaluation
    eval();
}

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv)) return 1;  // should exit.
        do_kitti_eval_error();
        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "Exit due to exception:\n"
                  << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}

using Matrix = mrpt::math::CMatrixDouble44;

static void parse_calib_line(
    const std::string& line, Eigen::Matrix<double, 3, 4>& M)
{
    MRPT_TRY_START

    std::istringstream ss(line);
    for (Eigen::Index r = 0; r < M.rows(); r++)
        for (Eigen::Index c = 0; c < M.cols(); c++)
        {
            if (!(ss >> M(r, c)))
            {
                THROW_EXCEPTION_FMT(
                    "Error parsing calib line: `%s`", line.c_str());
            }
        }
    MRPT_TRY_END
}
std::vector<Matrix> loadPoses_tum_format(
    const std::string& file_name, const std::string& calib_file, bool isGT)
{
    mrpt::poses::CPose3DInterpolator trajectory;

    if (!trajectory.loadFromTextFile_TUM(file_name))
        throw std::runtime_error(mrpt::format(
            "Error loading trajectory from: %s", file_name.c_str()));

    std::cout << "Parsed: " << file_name << "\n";
    std::cout << " - Loaded poses: " << trajectory.size() << "\n";

    // Transform poses: the file are poses of the vehicle (in Kitti datasets,
    // the reference is the Velodyne), but the GT files are given for the cam0!

    // Transform them:

    // Velodyne is the (0,0,0) of the vehicle.
    // image_0 pose wrt velo is "Tr":
    // From calibration files:

    auto cam_pose0 = mrpt::poses::CPose3D::Identity();

    if (!calib_file.empty())
    {
        auto calib = mrpt::containers::yaml::FromFile(calib_file);
        ASSERT_(calib.has("Tr"));

        Eigen::Matrix<double, 3, 4> Tr;
        parse_calib_line(calib["Tr"].as<std::string>(), Tr);

        auto Trh              = mrpt::math::CMatrixDouble44::Identity();
        Trh.block<3, 4>(0, 0) = Tr;

        // std::cout << "Original Trh= (velo wrt cam_0) \n" << Trh << "\n";
        // Inverse:
        Trh = Trh.inverse();
        // std::cout << "Inverted Trh= (cam_0 wrt velo) \n" << Trh << "\n";

        // Camera 0:
        cam_pose0 = mrpt::poses::CPose3D(Trh);
    }

    std::vector<Matrix> poses;
    const auto          n = trajectory.size();
    poses.resize(n);

    std::optional<std::ofstream> fKittiOut;

    if (!isGT && argSavePathKittiFormat.isSet())
    {
        const auto fil = argSavePathKittiFormat.getValue();
        fKittiOut.emplace();
        fKittiOut->open(fil);
        ASSERT_(*fKittiOut);

        std::cout << "= Exporting path in kitti format to: " << fil
                  << std::endl;
    }

    mrpt::poses::CPose3D pose0;
    size_t               i = 0;
    for (const auto& p : trajectory)
    {
        auto pose = mrpt::poses::CPose3D(p.second);
        pose      = pose + cam_pose0;

        if (i == 0) pose0 = pose;

        pose = pose - pose0;

        mrpt::math::CMatrixDouble44 HM;
        pose.getHomogeneousMatrix(HM);
        poses[i++] = HM;

        // save in kitti format?
        if (fKittiOut)
        {
            fKittiOut.value() << mrpt::format(
                "%lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg\n",  //
                HM(0, 0), HM(0, 1), HM(0, 2), HM(0, 3),  //
                HM(1, 0), HM(1, 1), HM(1, 2), HM(1, 3),  //
                HM(2, 0), HM(2, 1), HM(2, 2), HM(2, 3));
        }
    }

    return poses;
}

// -------------------------------------------------------
// Start of code adapted from the kitti-eval package
// -------------------------------------------------------

using namespace std;

const float   lengths[]   = {100, 200, 300, 400, 500, 600, 700, 800};
const int32_t num_lengths = 8;

struct errors
{
    int32_t first_frame;
    float   r_err;
    float   t_err;
    float   len;
    float   speed;
    errors(
        int32_t first_frame, float r_err, float t_err, float len, float speed)
        : first_frame(first_frame),
          r_err(r_err),
          t_err(t_err),
          len(len),
          speed(speed)
    {
    }
};

std::vector<Matrix> loadPoses(string file_name)
{
    vector<Matrix> poses;
    FILE*          fp = fopen(file_name.c_str(), "r");
    if (!fp) return poses;
    while (!feof(fp))
    {
        Matrix P = Matrix::Identity();
        if (fscanf(
                fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &P(0, 0),
                &P(0, 1), &P(0, 2), &P(0, 3), &P(1, 0), &P(1, 1), &P(1, 2),
                &P(1, 3), &P(2, 0), &P(2, 1), &P(2, 2), &P(2, 3)) == 12)
            poses.push_back(P);
    }
    fclose(fp);
    return poses;
}

vector<float> trajectoryDistances(vector<Matrix>& poses)
{
    vector<float> dist;
    dist.push_back(0);
    for (int32_t i = 1; i < poses.size(); i++)
    {
        Matrix P1 = poses[i - 1];
        Matrix P2 = poses[i];
        float  dx = P1(0, 3) - P2(0, 3);
        float  dy = P1(1, 3) - P2(1, 3);
        float  dz = P1(2, 3) - P2(2, 3);
        dist.push_back(dist[i - 1] + sqrt(dx * dx + dy * dy + dz * dz));
    }
    return dist;
}

int32_t lastFrameFromSegmentLength(
    vector<float>& dist, int32_t first_frame, float len)
{
    for (int32_t i = first_frame; i < dist.size(); i++)
        if (dist[i] > dist[first_frame] + len) return i;
    return -1;
}

inline float rotationError(Matrix& pose_error)
{
    float a = pose_error(0, 0);
    float b = pose_error(1, 1);
    float c = pose_error(2, 2);
    float d = 0.5 * (a + b + c - 1.0);
    return acos(max(min(d, 1.0f), -1.0f));
}

inline float translationError(Matrix& pose_error)
{
    float dx = pose_error(0, 3);
    float dy = pose_error(1, 3);
    float dz = pose_error(2, 3);
    return sqrt(dx * dx + dy * dy + dz * dz);
}

vector<errors> calcSequenceErrors(
    vector<Matrix>& poses_gt, vector<Matrix>& poses_result)
{
    // error vector
    vector<errors> err;

    // parameters
    int32_t step_size = 10;  // every second

    // pre-compute distances (from ground truth as reference)
    vector<float> dist = trajectoryDistances(poses_gt);

    // for all start positions do
    for (int32_t first_frame = 0; first_frame < poses_gt.size();
         first_frame += step_size)
    {
        // for all segment lengths do
        for (int32_t i = 0; i < num_lengths; i++)
        {
            // current length
            float len = lengths[i];

            // compute last frame
            int32_t last_frame =
                lastFrameFromSegmentLength(dist, first_frame, len);

            // continue, if sequence not long enough
            if (last_frame == -1) continue;

            // compute rotational and translational errors
            Matrix pose_delta_gt =
                (poses_gt[first_frame]).inverse() * poses_gt[last_frame];
            Matrix pose_delta_result = (poses_result[first_frame]).inverse() *
                                       poses_result[last_frame];
            Matrix pose_error = (pose_delta_result).inverse() * pose_delta_gt;
            float  r_err      = rotationError(pose_error);
            float  t_err      = translationError(pose_error);

            // compute speed
            float num_frames = (float)(last_frame - first_frame + 1);
            float speed      = len / (0.1 * num_frames);

            // write to file
            err.push_back(
                errors(first_frame, r_err / len, t_err / len, len, speed));
        }
    }

    // return error vector
    return err;
}

void saveSequenceErrors(vector<errors>& err, string file_name)
{
    // open file
    FILE* fp;
    fp = fopen(file_name.c_str(), "w");
    ASSERTMSG_(
        fp, mrpt::format("Could not open for writing: %s", file_name.c_str()));
    fprintf(fp, "%% first_frame rot_err trans_error len  speed\n");

    // write to file
    for (vector<errors>::iterator it = err.begin(); it != err.end(); it++)
        fprintf(
            fp, "%d %f %f %f %f\n", it->first_frame, it->r_err, it->t_err,
            it->len, it->speed);

    // close file
    fclose(fp);
}

void savePathPlot(
    vector<Matrix>& poses_gt, vector<Matrix>& poses_result, string file_name)
{
    // parameters
    int32_t step_size = 3;

    // open file
    FILE* fp = fopen(file_name.c_str(), "w");

    // save x/z coordinates of all frames to file
    for (int32_t i = 0; i < poses_gt.size(); i += step_size)
        fprintf(
            fp, "%f %f %f %f\n", poses_gt[i](0, 3), poses_gt[i](2, 3),
            poses_result[i](0, 3), poses_result[i](2, 3));

    // close file
    fclose(fp);
}

vector<int32_t> computeRoi(
    vector<Matrix>& poses_gt, vector<Matrix>& poses_result)
{
    float x_min = numeric_limits<int32_t>::max();
    float x_max = numeric_limits<int32_t>::min();
    float z_min = numeric_limits<int32_t>::max();
    float z_max = numeric_limits<int32_t>::min();

    for (vector<Matrix>::iterator it = poses_gt.begin(); it != poses_gt.end();
         it++)
    {
        float x = it->coeff(0, 3);
        float z = it->coeff(2, 3);
        if (x < x_min) x_min = x;
        if (x > x_max) x_max = x;
        if (z < z_min) z_min = z;
        if (z > z_max) z_max = z;
    }

    for (vector<Matrix>::iterator it = poses_result.begin();
         it != poses_result.end(); it++)
    {
        float x = it->coeff(0, 3);
        float z = it->coeff(2, 3);
        if (x < x_min) x_min = x;
        if (x > x_max) x_max = x;
        if (z < z_min) z_min = z;
        if (z > z_max) z_max = z;
    }

    float dx = 1.1 * (x_max - x_min);
    float dz = 1.1 * (z_max - z_min);
    float mx = 0.5 * (x_max + x_min);
    float mz = 0.5 * (z_max + z_min);
    float r  = 0.5 * max(dx, dz);

    vector<int32_t> roi;
    roi.push_back((int32_t)(mx - r));
    roi.push_back((int32_t)(mx + r));
    roi.push_back((int32_t)(mz - r));
    roi.push_back((int32_t)(mz + r));
    return roi;
}

void plotPathPlot(string dir, vector<int32_t>& roi, int32_t idx)
{
    // gnuplot file name
    char command[1024];
    char file_name[256];
    sprintf(file_name, "%02d.gp", idx);
    string full_name = dir + "/" + file_name;

    // create png + eps
    for (int32_t i = 0; i < 2; i++)
    {
        // open file
        FILE* fp = fopen(full_name.c_str(), "w");

        // save gnuplot instructions
        if (i == 0)
        {
            fprintf(fp, "set term png size 900,900\n");
            fprintf(fp, "set output \"%02d.png\"\n", idx);
        }
        else
        {
            fprintf(fp, "set term postscript eps enhanced color\n");
            fprintf(fp, "set output \"%02d.eps\"\n", idx);
        }

        fprintf(fp, "set size ratio -1\n");
        fprintf(fp, "set xrange [%d:%d]\n", roi[0], roi[1]);
        fprintf(fp, "set yrange [%d:%d]\n", roi[2], roi[3]);
        fprintf(fp, "set xlabel \"x [m]\"\n");
        fprintf(fp, "set ylabel \"z [m]\"\n");
        fprintf(
            fp,
            "plot \"%02d.txt\" using 1:2 lc rgb \"#FF0000\" title 'Ground "
            "Truth' w lines,",
            idx);
        fprintf(
            fp,
            "\"%02d.txt\" using 3:4 lc rgb \"#0000FF\" title 'Visual Odometry' "
            "w lines,",
            idx);
        fprintf(
            fp,
            "\"< head -1 %02d.txt\" using 1:2 lc rgb \"#000000\" pt 4 ps 1 lw "
            "2 title 'Sequence Start' w points\n",
            idx);

        // close file
        fclose(fp);

        // run gnuplot => create png + eps
        sprintf(command, "cd %s; gnuplot %s", dir.c_str(), file_name);
        system(command);
    }

    // create pdf and crop
    if (!argSkipFigures.isSet())
    {
        sprintf(
            command, "cd %s; ps2pdf %02d.eps %02d_large.pdf", dir.c_str(), idx,
            idx);
        system(command);
        sprintf(
            command,
            "bash -c 'cd %s; pdfcrop %02d_large.pdf %02d.pdf > /dev/null 2>&1'",
            dir.c_str(), idx, idx);
        system(command);
        sprintf(command, "cd %s; rm %02d_large.pdf", dir.c_str(), idx);
        system(command);
    }
}

void saveErrorPlots(
    vector<errors>& seq_err, string plot_error_dir, const char* prefix)
{
    // file names
    char file_name_tl[1024];
    sprintf(file_name_tl, "%s/%s_tl.txt", plot_error_dir.c_str(), prefix);
    char file_name_rl[1024];
    sprintf(file_name_rl, "%s/%s_rl.txt", plot_error_dir.c_str(), prefix);
    char file_name_ts[1024];
    sprintf(file_name_ts, "%s/%s_ts.txt", plot_error_dir.c_str(), prefix);
    char file_name_rs[1024];
    sprintf(file_name_rs, "%s/%s_rs.txt", plot_error_dir.c_str(), prefix);

    // open files
    FILE* fp_tl = fopen(file_name_tl, "w");
    FILE* fp_rl = fopen(file_name_rl, "w");
    FILE* fp_ts = fopen(file_name_ts, "w");
    FILE* fp_rs = fopen(file_name_rs, "w");

    // for each segment length do
    for (int32_t i = 0; i < num_lengths; i++)
    {
        float t_err = 0;
        float r_err = 0;
        float num   = 0;

        // for all errors do
        for (vector<errors>::iterator it = seq_err.begin(); it != seq_err.end();
             it++)
        {
            if (fabs(it->len - lengths[i]) < 1.0)
            {
                t_err += it->t_err;
                r_err += it->r_err;
                num++;
            }
        }

        // we require at least 3 values
        if (num > 2.5)
        {
            fprintf(fp_tl, "%f %f\n", lengths[i], t_err / num);
            fprintf(fp_rl, "%f %f\n", lengths[i], r_err / num);
        }
    }

    // for each driving speed do (in m/s)
    for (float speed = 2; speed < 25; speed += 2)
    {
        float t_err = 0;
        float r_err = 0;
        float num   = 0;

        // for all errors do
        for (vector<errors>::iterator it = seq_err.begin(); it != seq_err.end();
             it++)
        {
            if (fabs(it->speed - speed) < 2.0)
            {
                t_err += it->t_err;
                r_err += it->r_err;
                num++;
            }
        }

        // we require at least 3 values
        if (num > 2.5)
        {
            fprintf(fp_ts, "%f %f\n", speed, t_err / num);
            fprintf(fp_rs, "%f %f\n", speed, r_err / num);
        }
    }

    // close files
    fclose(fp_tl);
    fclose(fp_rl);
    fclose(fp_ts);
    fclose(fp_rs);
}

void plotErrorPlots(string dir, const char* prefix)
{
    char command[4096];

    // for all four error plots do
    for (int32_t i = 0; i < 4; i++)
    {
        // create suffix
        char suffix[16];
        switch (i)
        {
            case 0:
                sprintf(suffix, "tl");
                break;
            case 1:
                sprintf(suffix, "rl");
                break;
            case 2:
                sprintf(suffix, "ts");
                break;
            case 3:
                sprintf(suffix, "rs");
                break;
        }

        // gnuplot file name
        char file_name[1024];
        char full_name[2048];
        sprintf(file_name, "%s_%s.gp", prefix, suffix);
        sprintf(full_name, "%s/%s", dir.c_str(), file_name);

        // create png + eps
        for (int32_t j = 0; j < 2; j++)
        {
            // open file
            FILE* fp = fopen(full_name, "w");

            // save gnuplot instructions
            if (j == 0)
            {
                fprintf(
                    fp, "set term png size 500,250 font \"Helvetica\" 11\n");
                fprintf(fp, "set output \"%s_%s.png\"\n", prefix, suffix);
            }
            else
            {
                fprintf(fp, "set term postscript eps enhanced color\n");
                fprintf(fp, "set output \"%s_%s.eps\"\n", prefix, suffix);
            }

            // start plot at 0
            fprintf(fp, "set size ratio 0.5\n");
            fprintf(fp, "set yrange [0:*]\n");

            // x label
            if (i <= 1)
                fprintf(fp, "set xlabel \"Path Length [m]\"\n");
            else
                fprintf(fp, "set xlabel \"Speed [km/h]\"\n");

            // y label
            if (i == 0 || i == 2)
                fprintf(fp, "set ylabel \"Translation Error [%%]\"\n");
            else
                fprintf(fp, "set ylabel \"Rotation Error [deg/m]\"\n");

            // plot error curve
            fprintf(fp, "plot \"%s_%s.txt\" using ", prefix, suffix);
            switch (i)
            {
                case 0:
                    fprintf(fp, "1:($2*100) title 'Translation Error'");
                    break;
                case 1:
                    fprintf(fp, "1:($2*57.3) title 'Rotation Error'");
                    break;
                case 2:
                    fprintf(fp, "($1*3.6):($2*100) title 'Translation Error'");
                    break;
                case 3:
                    fprintf(fp, "($1*3.6):($2*57.3) title 'Rotation Error'");
                    break;
            }
            fprintf(fp, " lc rgb \"#0000FF\" pt 4 w linespoints\n");

            // close file
            fclose(fp);

            // run gnuplot => create png + eps
            sprintf(command, "cd %s; gnuplot %s", dir.c_str(), file_name);
            system(command);
        }

        // create pdf and crop
        if (!argSkipFigures.isSet())
        {
            sprintf(
                command, "cd %s; ps2pdf %s_%s.eps %s_%s_large.pdf", dir.c_str(),
                prefix, suffix, prefix, suffix);
            printf("Exec: %s\n", command);
            system(command);
            sprintf(
                command,
                "bash -c 'cd %s; pdfcrop %s_%s_large.pdf %s_%s.pdf > /dev/null "
                "2>&1'",
                dir.c_str(), prefix, suffix, prefix, suffix);
            system(command);
            sprintf(
                command, "cd %s; rm %s_%s_large.pdf", dir.c_str(), prefix,
                suffix);
            system(command);
        }
    }
}

void saveStats(vector<errors> err, string dir)
{
    float t_err           = 0;
    float r_err           = 0;
    float r_err_per_meter = 0;

    // for all errors do => compute sum of t_err, r_err
    for (vector<errors>::iterator it = err.begin(); it != err.end(); it++)
    {
        t_err += it->t_err;
        r_err += it->r_err;
        r_err_per_meter += it->r_err / it->len;
    }

    // open file
    printf(" === Saving stats to: %s\n", (dir + "/stats.txt").c_str());
    FILE* fp = fopen((dir + "/stats.txt").c_str(), "w");

    // save errors
    float num = err.size();
    fprintf(fp, "%% Overall error: trans_error (percent)  rot_error\n");
    fprintf(fp, "%f  %f\n", 100 * t_err / num, r_err / num);

    printf(
        "%% Overall error: trans_error(%%)  rot_error(rad/m) "
        "rot_error(deg/m)\n");
    printf(
        "%f  %f %f\n", 100 * t_err / num, r_err / num,
        (180.0 / M_PI) * r_err / num);

    // close file
    fclose(fp);
}

bool eval()  // string result_sha,Mail* mail)
{
    // ground truth and result directories
    string gt_dir = kitti_basedir + "/poses";

    string result_dir =
        mrpt::system::extractFileDirectory(arg_result_path.getValue());
    if (result_dir.empty()) result_dir = ".";

    std::cout << "Using as result_dir: " << result_dir << "\n";

    string error_dir      = result_dir + "/errors";
    string plot_path_dir  = result_dir + "/plot_path";
    string plot_error_dir = result_dir + "/plot_error";

    // create output directories
    if (!mrpt::system::directoryExists(error_dir))
        mrpt::system::createDirectory(error_dir);
    if (!mrpt::system::directoryExists(plot_path_dir))
        mrpt::system::createDirectory(plot_path_dir);
    if (!mrpt::system::directoryExists(plot_error_dir))
        mrpt::system::createDirectory(plot_error_dir);

    // total errors
    vector<errors> total_err;

    struct InfoPerSeq
    {
        bool        is_kitti     = true;
        int         kitti_seq_no = 0;
        std::string file_name;
        std::string kitti_gt_poses_file;
        std::string kitti_calib_file;

        std::string custom_gt_tum_file;
        std::string result_file;
    };

    std::vector<InfoPerSeq> seqs;

    if (arg_override_gt_file.isSet())
    {
        // custom ground truth TUM file(s):

        // multiple files?
        std::vector<std::string> gtFiles, resultFiles;
        mrpt::system::tokenize(arg_override_gt_file.getValue(), ",", gtFiles);
        mrpt::system::tokenize(arg_result_path.getValue(), ",", resultFiles);

        ASSERT_EQUAL_(gtFiles.size(), resultFiles.size());

        for (size_t i = 0; i < gtFiles.size(); i++)
        {
            auto& s = seqs.emplace_back();

            s.is_kitti           = false;
            s.custom_gt_tum_file = gtFiles[i];
            s.result_file        = resultFiles[i];
            s.file_name = mrpt::system::extractFileName(s.custom_gt_tum_file);
        }
    }
    else
    {
        // original KITTI dataset GT files:
        for (int32_t i : arg_seq.getValue())
        {
            auto& s = seqs.emplace_back();

            s.is_kitti     = true;
            s.kitti_seq_no = i;
            s.file_name    = mrpt::format("%02d.txt", i);
            s.result_file = mrpt::format(arg_result_path.getValue().c_str(), i);
            s.kitti_calib_file = mrpt::format(
                "%s/sequences/%02i/calib.txt", kitti_basedir.c_str(), i);
            s.kitti_gt_poses_file = gt_dir + "/" + s.file_name;
        }
    }

    // for all sequences do
    for (const auto& seq : seqs)
    {
        vector<Matrix> poses_result;
        vector<Matrix> poses_gt;

        if (seq.is_kitti)
        {
            // read ground truth and result poses
            poses_gt     = loadPoses(seq.kitti_gt_poses_file);
            poses_result = loadPoses_tum_format(
                seq.result_file, seq.kitti_calib_file, false);
        }
        else
        {
            poses_gt = loadPoses_tum_format(seq.custom_gt_tum_file, {}, false);
            poses_result = loadPoses_tum_format(seq.result_file, {}, false);
        }

        // plot status
        printf(
            "Processing: %s, poses: %ld/%ld\n", seq.file_name.c_str(),
            poses_result.size(), poses_gt.size());

        printf("-- poses_gt: %lu entries\n", poses_gt.size());
        printf("-- poses_result: %lu entries\n", poses_result.size());

        // check for errors
        if (poses_gt.size() == 0 || poses_result.size() != poses_gt.size())
        {
            printf(
                "ERROR: Couldn't read (all) poses of: %s\n",
                seq.file_name.c_str());
            return false;
        }

        // compute sequence errors
        vector<errors> seq_err = calcSequenceErrors(poses_gt, poses_result);
        saveSequenceErrors(seq_err, error_dir + "/" + seq.file_name);

        // add to total errors
        total_err.insert(total_err.end(), seq_err.begin(), seq_err.end());

        // for first half => plot trajectory and compute individual stats
        if (/*seq.kitti_seq_no <= 15 && */ !argSkipFigures.isSet())
        {
            // save + plot bird's eye view trajectories
            savePathPlot(
                poses_gt, poses_result, plot_path_dir + "/" + seq.file_name);
            vector<int32_t> roi = computeRoi(poses_gt, poses_result);
            plotPathPlot(plot_path_dir, roi, seq.kitti_seq_no);

            // save + plot individual errors
            saveErrorPlots(seq_err, plot_error_dir, seq.file_name.c_str());
            plotErrorPlots(plot_error_dir, seq.file_name.c_str());
        }
    }

    // save + plot total errors + summary statistics
    if (total_err.size() > 0)
    {
        char prefix[16];
        sprintf(prefix, "avg");
        saveErrorPlots(total_err, plot_error_dir, prefix);
        plotErrorPlots(plot_error_dir, prefix);
        saveStats(total_err, result_dir);
    }

    // success
    return true;
}
