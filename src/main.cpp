/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    main.cpp

    @version    0.1
    @author     Miika 'LehdaRi' Lehtimäki
    @date       2015-04-24

**/


#include "Mesh.hpp"
#include "Shader.hpp"
#include "Arm.hpp"

#include <iostream>
#include <SFML/Window.hpp>
#include <GL/glew.h>


int main()
{
    // create the window
    sf::Window window(sf::VideoMode(960, 720), "Robokaesi", sf::Style::Default, sf::ContextSettings(32));
    window.setFramerateLimit(60);

    // load resources, initialize the OpenGL states, ...
    glewInit();
    glCullFace(GL_CCW);
    glEnable(GL_DEPTH_TEST);

    try {
        Mesh jMesh0, jMesh1, jMesh2, jMesh3, jMesh4, jMesh5;
        jMesh0.loadFromObj("res/models/puma_link1.obj");
        jMesh1.loadFromObj("res/models/puma_link2.obj");
        jMesh2.loadFromObj("res/models/puma_link3.obj");
        jMesh3.loadFromObj("res/models/puma_link4.obj");
        jMesh4.loadFromObj("res/models/puma_link5.obj");
        jMesh5.loadFromObj("res/models/puma_link6.obj");

        Shader meshShader("res/shaders/VS_Simple.glsl", "res/shaders/FS_Simple.glsl");
        Shader jointShader("res/shaders/VS_Joint.glsl", "res/shaders/FS_Joint.glsl");

        Camera camera;
        camera.lookAt(Vector3Glf(80.0f, 80.0f, 55.0f), Vector3Glf(0.0f, 0.0f, 20.0f), Vector3Glf(0.0f, 0.0f, 1.0f));
        auto windowSize = window.getSize();
        camera.projection(0.25*PI, (float)windowSize.x/(float)windowSize.y, 1.0f, 500.0f);

        /*Joint j0(jointShader);
        Joint j1(jointShader);
        Joint j2(jointShader);
        Joint j3(jointShader);
        Joint j4(jointShader);
        Joint j5(jointShader);
        Joint j6(jointShader);*/


        /*j0.setDHParameters(0.0f, 26.45f, 0.0f, 0.0f);
        j1.setDHParameters(0.0f, -5.5, 0.0f, PI*0.5f);
        j2.setDHParameters(0.0f, 0.0f, 17.0f, 0.0f);*/

        Matrix4Glf jm0;
        jm0 <<  1.0f,   0.0f,   0.0f,   0.0f,
                0.0f,   1.0f,   0.0f,   0.0f,
                0.0f,   0.0f,   1.0f,   26.45f,
                0.0f,   0.0f,   0.0f,   1.0f;

        Matrix4Glf jm1;
        jm1 <<  0.0f,   0.0f,   1.0f,   5.5f,
                0.0f,   1.0f,   0.0f,   0.0f,
                -1.0f,  0.0f,   0.0f,   0.0f,
                0.0f,   0.0f,   0.0f,   1.0f;

        Matrix4Glf jm2; // move
        jm2 <<  1.0f,   0.0f,   0.0f,   17.0f,
                0.0f,   1.0f,   0.0f,   0.0f,
                0.0f,   0.0f,   1.0f,   0.0f,
                0.0f,   0.0f,   0.0f,   1.0f;

        Matrix4Glf jm3a; // move
        jm3a <<  1.0f,   0.0f,   0.0f,   17.05f,
                0.0f,   1.0f,   0.0f,   0.0f,
                0.0f,   0.0f,   1.0f,   0.0f,
                0.0f,   0.0f,   0.0f,   1.0f;

        Matrix4Glf jm3b; // rot y
        jm3b <<  0.0f,   0.0f,   1.0f,   0.0f,
                0.0f,   1.0f,   0.0f,   0.0f,
                -1.0f,  0.0f,   0.0f,   0.0f,
                0.0f,   0.0f,   0.0f,   1.0f;

        Matrix4Glf jm3 = jm3a*jm3b;

        Matrix4Glf jm4; // rot y
        jm4 <<  0.0f,   0.0f,   1.0f,   0.0f,
                0.0f,   1.0f,   0.0f,   0.0f,
                -1.0f,  0.0f,   0.0f,   0.0f,
                0.0f,   0.0f,   0.0f,   1.0f;

        Matrix4Glf jm5; // rot y & move
        jm5 <<  0.0f,   0.0f,   1.0f,   2.2f,
                0.0f,   1.0f,   0.0f,   0.0f,
                -1.0f,  0.0f,   0.0f,   0.0f,
                0.0f,   0.0f,   0.0f,   1.0f;


        std::vector<Matrix4Glf> jointMatrices;
        jointMatrices.push_back(jm0);
        jointMatrices.push_back(jm1);
        jointMatrices.push_back(jm2);
        jointMatrices.push_back(jm3);
        //jointMatrices.push_back(jm4);
        //jointMatrices.push_back(jm5);

        Arm arm(jointShader, jointMatrices);

        arm.setJointTheta(0, 0.0f);
        arm.solve(Vector3Glf(15.0f, 25.0f, 26.45f), 100);

        /*j1.applyJoint(j0);
        j2.applyJoint(j1);
        j3.applyJoint(j2);
        j4.applyJoint(j3);
        j5.applyJoint(j4);
        j6.applyJoint(j5);*/
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
/*
            jMesh0.render(meshShader, camera, Vector3Glf(0.35f, 0.33f, 0.30f));
            jMesh1.render(meshShader, camera, j1, Vector3Glf(0.80f, 0.75f, 0.55f));
            jMesh2.render(meshShader, camera, j2, Vector3Glf(0.80f, 0.75f, 0.55f));
            jMesh3.render(meshShader, camera, j3, Vector3Glf(0.80f, 0.75f, 0.55f));
            jMesh4.render(meshShader, camera, j4, Vector3Glf(0.45f, 0.43f, 0.40f));
            jMesh5.render(meshShader, camera, j5, Vector3Glf(0.22f, 0.23f, 0.24f));
*/
            glDisable(GL_DEPTH_TEST);

            arm.draw(camera);
/*            j0.draw(camera);
            j1.draw(camera);
            j2.draw(camera);
            j3.draw(camera);
            j4.draw(camera);
            j5.draw(camera);
            j6.draw(camera);*/
            glEnable(GL_DEPTH_TEST);

            /*arm.setJointTheta(0, 0.3f * t);
            arm.setJointTheta(1, 0.0f);
            arm.setJointTheta(2, PI/2.0f + t*0.1f);
            arm.setJointTheta(3, PI/2 + t*0.4);
            arm.setJointTheta(4, 0+t*0.1f);
            arm.setJointTheta(5, PI+t);*/
/*
            j1.applyJoint(j0);
            j2.applyJoint(j1);
            j3.applyJoint(j2);
            j4.applyJoint(j3);
            j5.applyJoint(j4);
            j6.applyJoint(j5);
*/
            t += 0.01;

            // end the current frame (internally swaps the front and back buffers)
            window.display();
        }

        // release resources...
    }
    catch (const char* e) { // resource loading error (most likely)
        printf("%s", e);
        return 1;
    }

    return 0;
}
