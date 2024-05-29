#include "ns3/core-module.h"
#include "ns3/rosns3-module.h"
#include "ns3/rosns3-helper.h"

using namespace ns3;

int main (int argc, char *argv[])
{
  // Créer une instance de ROSNS3Server sur le port 12345
  ROSNS3Server server(12345);

  // Démarrer le serveur
  if (!server.start()) {
    std::cerr << "Erreur lors du démarrage du serveur" << std::endl;
    return 1;
  }

  // Boucle principale
  while (server.server_running()) {
    // Vérifier si des données sont prêtes à être lues
    if (server.data_ready()) {
      // Obtenir les données
      recvdata_t data = server.get_data();

      // Afficher les données reçues
      std::cout << "Données reçues : " << data.buffer << std::endl;

      // Envoyer une réponse
      const char* response = "Réponse du serveur";
      server.send_data((uint8_t*)response, strlen(response) + 1);
    }
  }

  // Arrêter le serveur
  server.kill();

  return 0;
}