/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    main.cpp

    @version    0.1
    @author     Miika 'LehdaRi' Lehtimäki
    @date       2015-04-18

**/


#include "Mesh.hpp"
#include "Shader.hpp"

#include <SFML/Window.hpp>
#include <GL/glew.h>


int main()
{
    // create the window
    sf::Window window(sf::VideoMode(800, 600), "Robokäsi", sf::Style::Default, sf::ContextSettings(32));
    window.setFramerateLimit(60);

    // load resources, initialize the OpenGL states, ...
    glewInit();
    glCullFace(GL_CCW);
    glEnable(GL_DEPTH_TEST);

    Mesh joint0;
    joint0.loadFromObj("res/models/puma_link1.obj");

    Shader shader("res/shaders/VS_Simple.glsl", "res/shaders/FS_Simple.glsl");

    Camera camera;
    camera.lookAt(Vector3Glf(50.0f, 40.0f, 40.0f), Vector3Glf(0.0f, 0.0f, 10.0f), Vector3Glf(0.0f, 0.0f, 1.0f));
    auto windowSize = window.getSize();
    camera.projection(70.0f, (float)windowSize.x/(float)windowSize.y, 1.0f, 200.0f);

    float t = 0.0f;

    // run the main loop
    bool running = true;
    while (running)
    {
        // handle events
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                // end the program
                running = false;
            }
            else if (event.type == sf::Event::Resized)
            {
                // adjust the viewport when the window is resized
                glViewport(0, 0, event.size.width, event.size.height);
            }
        }

        // clear the buffers
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        joint0.render(shader, camera, Vector3Glf(0.35f, 0.33f, 0.30f));

        camera.lookAt(Vector3Glf(45.0f*sinf(t), 45.0f*cosf(t), 40.0f),
                      Vector3Glf(0.0f, 0.0f, 10.0f),
                      Vector3Glf(0.0f, 0.0f, 1.0f));
        t += 0.01;

        // end the current frame (internally swaps the front and back buffers)
        window.display();
    }

    // release resources...

    return 0;
}
