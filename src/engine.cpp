#include "include/engine.hpp"
#include "include/scene.hpp"
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>
#include <optional>

Engine::Engine(uint32_t w, uint32_t h, const char *title)
    : window(sf::VideoMode({w, h}), title) {
  window.setFramerateLimit(120);
  sf::View view = window.getDefaultView();
  window.setView(view);
}

void Engine::run(Scene &scene) {
  sf::Clock clock;

  float dt = 1.f / 60.f;

  while (window.isOpen()) {
    while (auto ev = window.pollEvent()) {
      // Closed
      if (ev->is<sf::Event::Closed>())
        window.close();
      scene.handle_event(*ev, window);

      // Escape
      if (const auto *key = ev->getIf<sf::Event::KeyPressed>()) {
        if (key->code == sf::Keyboard::Key::Escape) {
          window.close();
        }
      }
    }

    scene.update(dt, window);

    window.clear(sf::Color::Black);
    scene.render(window);
    window.display();
  }
}
