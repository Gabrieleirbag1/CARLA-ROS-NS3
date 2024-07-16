#include "ns3/core-module.h"
#include "ns3/rosns3-module.h"
#include "ns3/rosns3-helper.h"
#include "ns3/netanim-module.h"
#include <regex>

using namespace ns3;

int main (int argc, char *argv[])
{
  AnimationInterface anim("animation.xml");
  anim.SetMobilityPollInterval(Seconds(1));

  anim.SetStartTime(Seconds(150)); 
  anim.SetStopTime(Seconds(150));
  // Créer une instance de ROSNS3Server sur le port 12345
  ROSNS3Server server(12345);

  // Démarrer le serveur
  if (!server.start()) {
    std::cerr << "Erreur lors du démarrage du serveur" << std::endl;
    return 1;
  }

  while (server.server_running()) {

    if (server.data_ready()) {
      recvdata_t data = server.get_data();
      std::string data_string = data.buffer;
      std::cout << "Données reçues : " << data.buffer << std::endl;

      // Envoyer une réponse
      const char* response = "Réponse du serveur";
      server.send_data((uint8_t*)response, strlen(response) + 1);

      server.kill();

    }
      
  }

  server.kill();

  return 0;
}