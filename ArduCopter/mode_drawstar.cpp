#include "Copter.h"

#if MODE_DRAWSTAR_ENABLED

// init - initialise guided controller
bool ModeDrawStar::init(bool ignore_checks)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Ds Init");

    current_point = 0;

    generate_star_path();

    pos_control_start();

    return true;
}

// run - runs the guided controller
// should be called at 100hz or more
void ModeDrawStar::run()
{
    if (current_point < 6)
    {
        if (wp_nav->reached_wp_destination())
        { // 到达某个端点
            current_point++;
            wp_nav->set_wp_destination(star_points[current_point], false); // 将下一个航点位置设置为导航控制模块的目标位置
            gcs().send_text(MAV_SEVERITY_INFO, "Current waypoint: %d", current_point);
        }
    }
    else if ((current_point == 6) && wp_nav->reached_wp_destination())
    { // 五角星航线运行完成，自动进入Loiter模式
        gcs().send_text(MAV_SEVERITY_INFO, "Draw star finished, now go into loiter mode");
        set_mode(Mode::Number::LOITER, ModeReason::MISSION_END); // 切换到loiter模式
        return;
    }

    pos_control_run();
}

// initialise guided mode's position controller
void ModeDrawStar::pos_control_start()
{

    pos_control->init_xy_controller();
    pos_control->init_z_controller();

    wp_nav->wp_and_spline_init();

    wp_nav->set_wp_destination(star_points[current_point], false);

    auto_yaw.set_mode_to_default(false);
}

void ModeDrawStar::pos_control_run()
{
    if (is_disarmed_or_landed()) // 安全检测
    {
        make_safe_ground_handling(motors->get_interlock());
        return;
    }

    // 设置油门最大输出
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    wp_nav->update_wpnav();              // 计算航点加速度
    pos_control->update_xy_controller(); // 更新水平控制器,将加速度转换为姿态
    pos_control->update_z_controller();  // 更新高度控制器

    attitude_control->input_thrust_vector_heading( // 发送推力矢量和航向到姿态控制器
        pos_control->get_thrust_vector(),          // 推力矢量
        auto_yaw.get_heading());                   // 航向
}

void ModeDrawStar::generate_star_path()
{
    float radius_cm = 1000.0;

    wp_nav->get_wp_stopping_point(star_points[0]);

    star_points[1] = star_points[0] + Vector3f(1.0f, 0, 0) * radius_cm;
    star_points[2] = star_points[0] + Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * radius_cm;
    star_points[3] = star_points[0] + Vector3f(sinf(radians(18.0f)), cosf(radians(18.0f)), 0) * radius_cm;
    star_points[4] = star_points[0] + Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * radius_cm;
    star_points[5] = star_points[0] + Vector3f(-cosf(radians(36.0f)), sinf(radians(36.0f)), 0) * radius_cm;
    star_points[6] = star_points[1];
}
#endif
