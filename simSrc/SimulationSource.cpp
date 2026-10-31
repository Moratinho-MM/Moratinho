
#include "../lib/Labirinto.h"
#include "../lib/Ratao.h"
#include "../lib/wayCompiler.h"

#define TAMANHO_LABIRINTO 16
#define OBJETIVO_X 8
#define OBJETIVO_Y 8


int main(int argc, char const *argv[])
{
    Labirinto* meuLab = new Labirinto(TAMANHO_LABIRINTO, OBJETIVO_X, OBJETIVO_Y);
    
    
    meuLab->gerarLabirinto();
    
    
    Ratao* ratao = new Ratao(meuLab, 0, 0);

    
    ratao->moverAteOPonto(OBJETIVO_X, OBJETIVO_Y);
    //ratao->moverAteOPonto(0, 1);
    ratao->moverAteOPonto(0, TAMANHO_LABIRINTO-1);
    ratao->moverAteOPonto(TAMANHO_LABIRINTO-1, TAMANHO_LABIRINTO-1);
    ratao->moverAteOPonto(TAMANHO_LABIRINTO-1, 0);
    ratao->moverAteOPonto(OBJETIVO_X, OBJETIVO_Y);
    ratao->moverAteOPonto(0, 0);
    //ratao->moverAteOPonto(1, 0);
    //ratao->moverAteOPonto(OBJETIVO, OBJETIVO);
    //ratao->moverAteOPonto(OBJETIVO, OBJETIVO);
    ratao->moverAteOPonto(OBJETIVO_X, OBJETIVO_Y);
    ratao->moverAteOPonto(0, 0);
    ratao->olharPara(NORTE);
    ratao->rushar(compilarCaminho(
        ratao->verDirecao(), 
        meuLab, 
        ratao->verPosicao().first,
        ratao->verPosicao().second, 
        OBJETIVO_X, OBJETIVO_Y));

    return 0;
}
