
#include "sphere_collision.hpp"

#include <random>
#include <chrono>

#ifdef SCENE_SPHERE_COLLISION

using namespace vcl;



void scene_model::frame_draw(std::map<std::string,GLuint>& , scene_structure& scene, gui_structure& )
{
    float dt = 0.02f * timer.scale;
    timer.update();

    set_gui();

    //Add 10 spheres
    if (gui_scene.add_multiple_spheres){
        for (size_t i  = 0; i < 10; i++)
        {
            create_new_particle();
        }
    }

    create_new_particle();

    //get nb divisions from gui
    divisions = (size_t)gui_scene.divisions;

    auto t1 = std::chrono::high_resolution_clock::now();
    compute_time_step(dt);
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

    printf("execution time : %u\n",duration);

    display_particles(scene);

    std::vector<vec3> borders_segments = {{-1,-1,-1},{1,-1,-1}, {1,-1,-1},{1,1,-1}, {1,1,-1},{-1,1,-1}, {-1,1,-1},{-1,-1,-1},
                                          {-1,-1,1} ,{1,-1,1},  {1,-1,1}, {1,1,1},  {1,1,1}, {-1,1,1},  {-1,1,1}, {-1,-1,1},
                                          {-1,-1,-1},{-1,-1,1}, {1,-1,-1},{1,-1,1}, {1,1,-1},{1,1,1},   {-1,1,-1},{-1,1,1}};

    std::vector<vec3>new_borders_segments;

    //compute new borders to draw
    for (size_t i = 0; i < 24; i++)
        new_borders_segments.push_back(borders_segments[i]+translation);

    borders = segments_gpu(new_borders_segments);
    borders.uniform.color = {0,0,0};
    borders.shader = shader_border;

    draw(borders, scene.camera);

}

void plane_collision(particle_structure& particle, const vec3& a, const vec3& n)
{
    float alpha = 0.9;
    float beta = 0.9;

    float scal = dot(particle.p-a, n);
    if (scal <= particle.r)
    {
        vec3 v_orth = dot(particle.v, n)*n;
        vec3 v_parr = particle.v - v_orth;

        particle.v = alpha*v_parr - beta*v_orth;

        float d = particle.r - dot((particle.p - a),n);

        particle.p = particle.p + d*n;
    }
}

void sphere_collision(particle_structure& p1, particle_structure& p2)
{
    float mass = 1;
    float friction = 0.5;

    const float diff_norm = norm(p1.p-p2.p);

    if (diff_norm <= p1.r + p2.r)
    {
            const vec3 u = (p1.p - p2.p)/diff_norm;
            float j = mass * dot((p2.v - p1.v),u);

            const vec3 J = j*u;

            //if frottement
            float epsilon = 0.001;
            if (j < epsilon)
            {
                p1.v = friction*p1.v;
                p2.v = friction*p2.v;
            }
            else //normal impact
            {
                p1.v = p1.v + J;
                p2.v = p2.v - J;
            }

            //replace the spheres
            float d = p1.r + p2.r - diff_norm + 0.001;
            p1.p = p1.p + (d/2)*u;
            p2.p = p2.p - (d/2)*u;
    }
}

void scene_model::compute_time_step(float dt)
{
    // Set forces

    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
        particles[k].f = down * 9.81;


    // Integrate position and speed of particles through time
    for(size_t k=0; k<N; ++k) {
        particle_structure& particle = particles[k];
        vec3& v = particle.v;
        vec3& p = particle.p;
        vec3 const& f = particle.f;

        v = (1-0.9f*dt) * v + dt * f; // gravity + friction force
        p = p + dt * v;
    }

    //udate the neighbours grid
    update_grid();

    const size_t grid_size = grid.size();
    for (size_t i = 0; i < grid_size; i++)
    {
        size_t G = grid[i].size();
        if (G == 0)
            continue;

        //calculate indices of surounding blocks from the grid to compute collisions near the sides of the sub block
        std::vector<size_t long> indices = {i-1,i+1,i+divisions,i-divisions,i+divisions*divisions,i-divisions*divisions};

        for (size_t p = 0; p < G; p++)
        {
            //collisions within the block
            particle_structure& particle1  = particles[grid[i][p].index];
            for (size_t q = 0; q < G; q++)
            {
                if (q == p)
                    continue;

                particle_structure& particle2  = particles[grid[i][q].index];
                sphere_collision(particle1, particle2);
            }


            //collisions with surrounding blocks
            for (size_t w = 0; w < 8; w++)
            {
                int index = indices[w];

                if (index > 0 && index < grid_size)
                {
                    const size_t G2 = grid[index].size();
                    for (size_t x = 0; x < G2; x++)
                    {
                        particle_structure& particle2  = particles[grid[index][x].index];
                        sphere_collision(particle1, particle2);
                    }
                }
            }
        }
    }

    //generate the planes of the box
    const vec3 a1 = vec3(0,-1,0)+translation;
    const vec3 n1 = vec3(0,1,0);
    const vec3 a2 = vec3(0,1,0) + translation;
    const vec3 n2 = vec3(0,-1,0);
    const vec3 a3 = vec3(1,0,0) + translation;
    const vec3 n3 = vec3(-1,0,0);
    const vec3 a4 = vec3(-1,0,0) + translation;
    const vec3 n4 = vec3(1,0,0);
    const vec3 a5 = vec3(0,0,1) + translation;
    const vec3 n5 = vec3(0,0,-1);
    const vec3 a6 = vec3(0,0,-1) + translation;
    const vec3 n6 = vec3(0,0,1);


    //compute plane collisions
    for(size_t k=0; k<N; ++k)
    {
        particle_structure& particle = particles[k];

        plane_collision(particle,a1,n1);
        plane_collision(particle,a2,n2);
        plane_collision(particle,a3,n3);
        plane_collision(particle,a4,n4);
        plane_collision(particle,a5,n5);
        plane_collision(particle,a6,n6);
    }

}


void scene_model::create_new_particle()
{
    // Emission of new particle if needed
    timer.periodic_event_time_step = gui_scene.time_interval_new_sphere;
    const bool is_new_particle = timer.event;
    static const std::vector<vec3> color_lut = {{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1}};

    if( is_new_particle && gui_scene.add_sphere)
    {
        particle_structure new_particle;

        new_particle.r = 0.02f;
        new_particle.c = color_lut[int(rand_interval()*color_lut.size())];

        // Initial position
        new_particle.p = vec3(0,0,0);

        // Initial speed
        const float theta = rand_interval(0, 2*3.14f);
        new_particle.v = vec3( 2*std::cos(theta), 5.0f, 2*std::sin(theta));

        new_particle.index = particles.size();
        particles.push_back(new_particle);
    }
}

void scene_model::update_grid()
{
    //clear the precedent grid
    for (size_t i = 0; i < grid.size(); i++)
    {
        grid[i].clear();
    }

    const size_t N = particles.size();

    for (size_t i = 0; i < N; i++)
    {
        const particle_structure &particle = particles[i];


        //calculate position of the particle
        size_t indx = (particle.p[0]+1)/width * divisions;
        size_t indy = (particle.p[1]+1)/width * divisions;
        size_t indz = (particle.p[2]+1)/width * divisions;

        //correct its position if it's out of the box
        if (indx == divisions)
            indx -=1;
        if (indy == divisions)
            indy -=1;
        if (indz == divisions)
            indz -=1;
        if (indx > divisions)
            indx = 0;
        if (indy > divisions)
            indy = 0;
        if (indz > divisions)
            indz = 0;

        //push in the grid
        size_t new_index = indx + indy * divisions + indz * divisions * divisions;

        grid[new_index].push_back(particle);
    }
}

void scene_model::display_particles(scene_structure& scene)
{
    const size_t N = particles.size();
    printf("Sphere number : %u\n", N);
    for(size_t k=0; k<N; ++k)
    {
        const particle_structure& part = particles[k];

        sphere.uniform.transform.translation = part.p;
        sphere.uniform.transform.scaling = part.r;
        sphere.uniform.color = part.c;

        vec3 down_world = {0,-1,0};
        down = scene.camera.orientation * down_world;
        translation = scene.camera.translation;

        draw(sphere, scene.camera);
    }
}




void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{
    sphere = mesh_drawable( mesh_primitive_sphere(1.0f));
    sphere.shader = shaders["mesh"];

    std::vector<vec3> borders_segments = {{-1,-1,-1},{1,-1,-1}, {1,-1,-1},{1,1,-1}, {1,1,-1},{-1,1,-1}, {-1,1,-1},{-1,-1,-1},
                                          {-1,-1,1} ,{1,-1,1},  {1,-1,1}, {1,1,1},  {1,1,1}, {-1,1,1},  {-1,1,1}, {-1,-1,1},
                                          {-1,-1,-1},{-1,-1,1}, {1,-1,-1},{1,-1,1}, {1,1,-1},{1,1,1},   {-1,1,-1},{-1,1,1}};
    borders = segments_gpu(borders_segments);
    borders.uniform.color = {0,0,0};
    borders.shader = shaders["curve"];
    shader_border = shaders["curve"];

    //Generate 3d spatial grid on one dimension
    for (size_t i = 0; i <= divisions * divisions * divisions; i++)
    {
        grid.push_back(std::vector<particle_structure>());
    }
}



void scene_model::set_gui()
{
    // Can set the speed of the animation
    ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
    ImGui::SliderFloat("Interval create sphere", &gui_scene.time_interval_new_sphere, 0.05f, 2.0f, "%.2f s");
    ImGui::SliderInt("Divisions", &gui_scene.divisions,1,100,"%d divs");
    ImGui::Checkbox("Add sphere", &gui_scene.add_sphere);
    ImGui::Checkbox("Add 10 spheres", &gui_scene.add_multiple_spheres);

    bool stop_anim  = ImGui::Button("Stop"); ImGui::SameLine();
    bool start_anim = ImGui::Button("Start");

    if(stop_anim)  timer.stop();
    if(start_anim) timer.start();
}





#endif
