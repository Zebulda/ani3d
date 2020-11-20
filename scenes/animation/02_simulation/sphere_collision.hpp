#pragma once

#include "main/scene_base/base.hpp"
#include <memory>

#ifdef SCENE_SPHERE_COLLISION

// Structure of a particle
struct particle_structure
{
    vcl::vec3 p; // Position
    vcl::vec3 v; // Speed
    vcl::vec3 f; // Forces

    vcl::vec3 c; // Color
    float r;     // Radius

    size_t index;
};

struct gui_scene_structure
{
    bool add_sphere = true;
    bool add_multiple_spheres = false;
    float time_interval_new_sphere = 5.5f;
    int divisions = 10;
};

struct scene_model : scene_base
{
    //width of the box
    size_t width = 2;
    //number of divisions
    size_t divisions = 100;

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);

    void set_gui();

    void compute_time_step(float dt);
    void create_new_particle();
    void display_particles(scene_structure& scene);
    void update_grid();

    std::vector<particle_structure> particles;

    std::vector<std::vector<particle_structure>> grid;

    vcl::mesh_drawable sphere;      // Visual display of particles
    vcl::segments_drawable borders; // Visual display of borders

    vcl::timer_event timer;
    gui_scene_structure gui_scene;

    //values of the camera
    vcl::vec3 down;
    vcl::vec3 translation;

    //keep the shader to draw the box
    int shader_border;
};






#endif
