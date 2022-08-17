#include <SFML/Graphics.hpp>
#include<vector>
#include<iostream>

#define SPRING_DAMPER  0.1f //Percentage of energy spring looses due to heat/sound/ext
#define SPRING_CONSTANT  120.f
#define SPRING_REST_LENGTH  40.f
#define SPRING_COLOR  sf::Color::White

#define PARTICLE_xOFFSET  200.f
#define PARTICLE_RADIUS  0.1f
#define PARTICLE_MASS  4.f
#define PARTICLE_X_DISPLACEMENT  30u
#define PARTICLE_Y_DISPLACEMENT  30u
#define PARTICLES_ON_X  5u
#define PARTICLES_ON_Y  5u
#define PARTICLE_MESH_FILL_COLOR sf::Color::Blue
#define PARTICLE_MESH_OUTLINE_COLOR sf::Color::White
#define PARTICLE_MESH_OUTLINE_THICKNESS 3.f

#define RECT_WIDTH  1200.f
#define RECT_HEIGHT  50.f
#define RECT_xPOS  0.f
#define RECT_yPOS  200.f
#define RECT_ROTATION  60.f

#define START_SIMULATION_KEY sf::Keyboard::S
#define APPLY_HIGH_GRAVITY_KEY sf::Keyboard::G
#define APPLY_WIND_KEY sf::Keyboard::W
#define DRAW_MESH_KEY sf::Keyboard::O

#define GRAVITY sf::Vector2f(0.0f, 9.8f )
#define HIGH_GRAVITY_MAGNITUDE 2.f
#define WIND sf::Vector2f( 10.0f, -9.8f )
#define FRICTION 0.98f

namespace sf
{
    namespace Vector
    {
        inline sf::Vector2f Multiply(const sf::Vector2f& v1, const sf::Vector2f& v2)
        {
            return sf::Vector2f(v1.x * v2.x, v1.y * v2.y);
        }

        inline float Magnitude(const sf::Vector2f& v)
        {
            return static_cast<float>(sqrt(pow(v.x, 2) + pow(v.y, 2)));
        }

        inline sf::Vector2f Normalize(const sf::Vector2f& v)
        {
            float m = Magnitude(v);
            if (m == 0)
                return { 0, 0 };

            return v / Magnitude(v);
        }

    }
}
struct Particle
{
    explicit Particle(const sf::Vector2f& aPosition, float aMass) : P(aPosition), M(aMass) {};
    Particle() = default;
    ~Particle() = default;

    sf::Vector2f P;//Position of the particle
    float M;//Mass of the particle

    sf::Vector2f F;//Force on the particle
    sf::Vector2f V;//Velocity of the particle

    bool FXD{false};//Is the particle fixed in place (STATIC)
};


struct Spring
{
    explicit Spring(float aSpringConstant, float aRestLength, Particle* aParticleA, Particle* aParticleB) : k(aSpringConstant), restLength(aRestLength), A(aParticleA), B(aParticleB) {};
    Spring() = default;
    ~Spring() = default;

    void Simulate()
    {
        //F = -k*x

        sf::Vector2f vAB = B->P - A->P;//define vector from A to B
        float ABmag = sf::Vector::Magnitude(vAB);//find magnitude of AB


        float springDisplacement =ABmag - restLength;//find spring displacement
        float forceMagnitude = -k * (springDisplacement - SPRING_DAMPER * springDisplacement);//calculate the magnitude of the spring force
        sf::Vector2f Fs = sf::Vector::Normalize(vAB) * forceMagnitude;//get a normal vector AB and apply force magnitude to get spring force vector

        //make A and B move towards or away from eachother bassed on spring compression or extension
        A->F -= Fs;
        B->F += Fs;
    }

    void DrawSpring(sf::RenderWindow& window)
    {
        sf::VertexArray vertz(sf::Lines, 2);
        vertz[0].position = A->P;
        vertz[0].color = SPRING_COLOR;

        vertz[1].position = B->P;
        vertz[1].color = SPRING_COLOR;

        window.draw(vertz);
    }

    float k;//the spring constant
    float restLength;//the spring rest length

    Particle* A;//pointer to mass points either side of spring
    Particle* B;
};



int main()
{
    sf::RenderWindow window(sf::VideoMode(1920, 1080), "Soft Body Simulation", sf::Style::Fullscreen);
    window.setFramerateLimit(60u);
    float dt = 1.f / 60.f;

    //a visual representation of particles
    sf::CircleShape particleShape(PARTICLE_RADIUS);
    particleShape.setFillColor(sf::Color::Red);
    particleShape.setOrigin({ PARTICLE_RADIUS, PARTICLE_RADIUS });

    //a static collidable rect
    sf::RectangleShape r({ RECT_WIDTH, RECT_HEIGHT });
    r.setPosition(RECT_xPOS, RECT_yPOS);
    r.rotate(RECT_ROTATION);

    //create particle mesh
    std::vector<Particle> particles;
    particles.resize(PARTICLES_ON_Y * PARTICLES_ON_X);

    for (int i = 0; i < PARTICLES_ON_Y; i++)
    {
        for (int j = 0; j < PARTICLES_ON_X; j++)
        {
            int particleIndex = i * PARTICLES_ON_X + j;

            float
                xPos = PARTICLE_xOFFSET + PARTICLE_RADIUS + static_cast<float>(j) * static_cast<float>(PARTICLE_X_DISPLACEMENT),
                yPos = static_cast<float>(PARTICLE_RADIUS + static_cast<float>(i) * static_cast<float>(PARTICLE_Y_DISPLACEMENT));

            particles[particleIndex].P = {xPos, yPos};
            particles[particleIndex].M = PARTICLE_MASS;
        }
    }

    //create shape to represent softbody
    sf::ConvexShape softBodyShape;
    constexpr unsigned int meshFramePointCount = PARTICLES_ON_X * 2 + (PARTICLES_ON_Y - 2) * 2;
    softBodyShape.setPointCount(meshFramePointCount);
    softBodyShape.setFillColor(PARTICLE_MESH_FILL_COLOR);
    softBodyShape.setOutlineColor(PARTICLE_MESH_OUTLINE_COLOR);
    softBodyShape.setOutlineThickness(PARTICLE_MESH_OUTLINE_THICKNESS);
    std::vector<int> meshFrameIndices;
    meshFrameIndices.resize(meshFramePointCount);

    int vertexNum = 0;
    //top x
    for (int j = 0; j < PARTICLES_ON_X; j++)
    {
        meshFrameIndices[vertexNum] = j;
        vertexNum++;
    }
    //right y
    for (int i = 1; i < PARTICLES_ON_Y; i++)
    {
        meshFrameIndices[vertexNum] = (PARTICLES_ON_X + i * PARTICLES_ON_X) - 1;
        vertexNum++;
    }

    //bottom x
    for (int j = 1; j < PARTICLES_ON_X; j++)
    {
        meshFrameIndices[vertexNum] = (PARTICLES_ON_X * PARTICLES_ON_Y - 1) - j;
        vertexNum++;
    }

    //left y
    for (int i = 1; i < PARTICLES_ON_Y - 1; i++)
    {
        meshFrameIndices[vertexNum] = PARTICLES_ON_X * (PARTICLES_ON_Y - i);
        vertexNum++;
    }

    //create spring network
    std::vector<Spring> springs;
    springs.resize(((PARTICLES_ON_X -1) * (PARTICLES_ON_Y - 1) * 4) + (PARTICLES_ON_Y -1) + (PARTICLES_ON_X - 1));

    int springNum = 0;

    for (int i = 0; i < PARTICLES_ON_Y; i++)
    {
        for (int j = 0; j < PARTICLES_ON_X; j++)
        {
            int particleIndex = j + PARTICLES_ON_X * i;

            
            if (j + 1 < PARTICLES_ON_X)//if particle on right side of current particle
            {
                springs[springNum].k = SPRING_CONSTANT;
                springs[springNum].restLength = SPRING_REST_LENGTH;
                springs[springNum].A = &particles[particleIndex];
                springs[springNum].B = &particles[particleIndex + 1];
                
                springNum++;
            }
           
            if (i + 1 < PARTICLES_ON_Y)
            {
                springs[springNum].k = SPRING_CONSTANT;
                springs[springNum].restLength = SPRING_REST_LENGTH;
                springs[springNum].A = &particles[particleIndex];
                springs[springNum].B = &particles[particleIndex + static_cast<int>(PARTICLES_ON_X)];

                springNum++;
            }

            if (j + 1 < PARTICLES_ON_X && i + 1 < PARTICLES_ON_Y)
            {
                springs[springNum].k = SPRING_CONSTANT;
                springs[springNum].restLength = SPRING_REST_LENGTH;
                springs[springNum].A = &particles[particleIndex];
                springs[springNum].B = &particles[particleIndex + static_cast<int>(PARTICLES_ON_X + 1)];

                springNum++;

                springs[springNum].k = SPRING_CONSTANT;
                springs[springNum].restLength = SPRING_REST_LENGTH;
                springs[springNum].A = &particles[particleIndex + 1];
                springs[springNum].B = &particles[particleIndex + static_cast<int>(PARTICLES_ON_X)];

                springNum++;
            }
        }
    }

    //simulation loop
    while (window.isOpen())
    {
        /*
        UPDATES
        */
        sf::Event event;
        while (window.pollEvent(event))
        {
            if(sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
                window.close();
        }

        static bool startSimulation = false;//start simulation on users request not as soon as program starts
        
        if (sf::Keyboard::isKeyPressed(START_SIMULATION_KEY))//check if should start simulation
            startSimulation = true;
        
        if (startSimulation)
        {
            //reset the force of the particles
            for (Particle& particle : particles)
                particle.F = { 0, 0 };

            //apply spring forces to particles
            for (Spring& spring : springs)
                spring.Simulate();

            bool applyHigherGravity = false;

            if (sf::Keyboard::isKeyPressed(APPLY_HIGH_GRAVITY_KEY))
                applyHigherGravity = true;

            //loop particles
            for (Particle& particle : particles)
            {
                //apply gravity
                if (applyHigherGravity)
                    particle.F += GRAVITY * HIGH_GRAVITY_MAGNITUDE;
                else
                    particle.F += GRAVITY;

                if(sf::Keyboard::isKeyPressed(APPLY_WIND_KEY))
                    particle.F += WIND;

                //If particle not locked in place
                if (!particle.FXD)
                {
                    //calculate velocity derivative
                    sf::Vector2f particleV_ = particle.V +  particle.F / particle.M;
                    particleV_ *= FRICTION;

                    //calculate position derivative
                    sf::Vector2f particleP_ = particle.P + particleV_ * dt;

                    //if position derivative not colliding with object in scene
                    if (!r.getLocalBounds().contains(r.getInverseTransform() * particleP_))
                    {
                        //set derivatives to current positions
                        particle.V = particleV_;
                        particle.P = particleP_;
                    }
                }
                //position correction if is y + particle radius > screenHeight
                if (particle.P.y > window.getSize().y - PARTICLE_RADIUS)
                    particle.P.y = window.getSize().y - PARTICLE_RADIUS;
            }
        }

        window.clear();

        bool drawMesh = true;

        if (sf::Keyboard::isKeyPressed(DRAW_MESH_KEY))
            drawMesh = false;

        if (drawMesh)
        {
            //render everything mesh related

            for (Spring& spring : springs)
                spring.DrawSpring(window);

            for (Particle& particle : particles)
            {
                particleShape.setPosition(particle.P);
                window.draw(particleShape);
            }
        }
        else
        {
            //render the soft body

            for (int i = 0; i < meshFramePointCount; i++)
                softBodyShape.setPoint(i, particles[meshFrameIndices[i]].P);

            window.draw(softBodyShape);

        }

        window.draw(r);
        window.display();

    }

    return 0;
}