# Analise de arquitetura PX4 para tethered drone

## Contexto

O projeto atual simula um drone conectado a um tether/cabo no Gazebo, com instrumentacao em ROS 2 para medir angulos do cabo, tensao, metricas de controle e trajetorias de teste. A arquitetura atual usa um controlador proprio em ROS 2 enviando comandos de velocidade para um plugin de controle simplificado do Gazebo.

A alternativa analisada e migrar gradualmente a parte de drone/controlador para PX4 SITL, mantendo no projeto atual os componentes especificos do tether:

- geracao e parametrizacao do cabo;
- ponto de ancoragem;
- junta de conexao cabo-drone;
- sensor de azimute/elevacao;
- metricas e scripts de teste;
- documentacao e casos experimentais.

## Recomendacao

Recomendacao principal:

```text
ADOTAR ARQUITETURA HIBRIDA
```

Isto significa manter a arquitetura atual como baseline de depuracao e criar, em paralelo, uma trilha PX4 SITL para substituir gradualmente apenas o drone/controlador.

Arquitetura atual:

```text
controlador ROS 2 proprio
  -> cmd_vel
  -> Gazebo MulticopterVelocityControl
  -> modelo simplificado do drone
  -> tether
```

Arquitetura alvo com PX4:

```text
PX4 SITL
  -> controladores PX4
  -> atuadores/motores simulados
  -> modelo PX4/Gazebo do drone
  -> junta de conexao
  -> tether
  -> ancora
```

O ROS 2 continua importante, mas muda de papel. Ele deixa de ser necessariamente o controlador principal do drone e passa a atuar como:

- camada de instrumentacao;
- publicador/coletor de metricas;
- supervisor de testes;
- interface de comandos de alto nivel ou offboard, se necessario;
- ponte para analise dos angulos do tether.

## Por que nao migrar tudo de uma vez

PX4 traria um controlador mais realista, proximo do que sera usado em Pixhawk, mas tambem adiciona complexidade:

- necessidade de lidar com arming, modos de voo e failsafes;
- dependencia da ponte PX4/ROS 2 via uXRCE-DDS;
- necessidade de adaptar um modelo Gazebo compativel com PX4;
- maior complexidade para depurar se um problema vem do tether, do modelo, do estimador ou do controlador PX4;
- possivel queda de desempenho com tether discretizado em muitos segmentos.

Como ainda existem questoes fisicas e geometricas no tether, a migracao completa agora misturaria muitos fatores ao mesmo tempo. A abordagem hibrida permite comparar:

```text
mesmo tether + controlador atual
mesmo tether + PX4 SITL
```

Isso ajuda a separar problemas do cabo de problemas do controlador.

## Fork, clone e upstream

Ha tres conceitos diferentes que costumam se confundir:

### Clone

Um clone e uma copia local de um repositorio Git.

Exemplo:

```bash
git clone https://github.com/PX4/PX4-Autopilot.git
```

Isso cria uma copia no seu computador. Um clone local sozinho nao cria um novo repositorio seu no GitHub.

### Fork

Um fork e uma copia do repositorio dentro da sua conta do GitHub.

Exemplo conceitual:

```text
PX4/PX4-Autopilot
  -> seu_usuario/PX4-Autopilot
```

O fork permite que voce tenha branches proprios, faca push e mantenha alteracoes sem modificar o repositorio oficial do PX4.

### Upstream

Upstream normalmente e o repositorio original do projeto.

Em um clone do seu fork, a configuracao ideal costuma ser:

```text
origin   = seu fork no GitHub
upstream = repositorio oficial PX4/PX4-Autopilot
```

Exemplo:

```bash
git remote -v
```

Resultado esperado:

```text
origin    git@github.com:seu_usuario/PX4-Autopilot.git
upstream  git@github.com:PX4/PX4-Autopilot.git
```

## O PX4 precisa ficar totalmente sincronizado?

Nao no sentido de que voce precise atualizar todos os dias. Mas ele precisa ter uma base conhecida e controlada.

O fluxo recomendado e:

1. escolher uma versao base do PX4;
2. registrar essa versao no projeto do tethered drone;
3. fazer seus testes sobre essa versao;
4. atualizar para uma versao mais nova apenas quando houver motivo;
5. validar novamente os testes apos cada atualizacao.

Para sincronizar com o PX4 oficial:

```bash
git fetch upstream
git checkout main
git merge upstream/main
```

Ou, se estiver usando uma branch de integracao:

```bash
git fetch upstream
git checkout main-sync
git merge upstream/main
git push origin main-sync
```

Depois, sua branch do tethered drone pode ser atualizada:

```bash
git checkout tethered-drone
git rebase main-sync
```

Isso nao e automatico. Voce decide quando sincronizar.

## Como o GitHub armazena um projeto grande como PX4

PX4 e um repositorio grande porque contem muitos arquivos, historico longo e submodulos. Existem alguns pontos importantes:

- O Git controla historico, nao apenas o estado atual dos arquivos.
- Um clone normal baixa o historico necessario do repositorio.
- Um fork no GitHub nao duplica necessariamente todo o armazenamento fisico de forma ingenua; o GitHub consegue compartilhar objetos internos entre fork e repositorio original.
- No seu computador, cada clone local ocupa espaco proprio.
- Se voce fizer dois clones completos do PX4, tera duas copias locais grandes.
- Para economizar espaco local, e possivel usar `git worktree`.

Tambem e possivel fazer clones mais leves:

```bash
git clone --depth 1 https://github.com/PX4/PX4-Autopilot.git
```

Mas para desenvolvimento serio isso pode atrapalhar merges, rebases e depuracao historica. Para um projeto de pesquisa com integracao de longo prazo, um clone completo costuma ser mais seguro.

## Situacao do tailsitter

O projeto tailsitter ja possui uma estrutura autocontida, com o PX4 dentro do diretorio do proprio projeto.

Isso tem vantagens:

- reproduzibilidade local;
- tudo que o projeto precisa esta dentro de uma arvore;
- menos dependencia de paths externos.

Mas tambem tem desvantagens:

- pode duplicar uma copia grande do PX4;
- dificulta compartilhar a mesma instalacao PX4 com outro projeto;
- pode misturar alteracoes especificas do tailsitter com alteracoes gerais no PX4;
- aumenta o risco de divergencia entre os projetos.

Como essa estrutura ja existe, nao e ideal reorganizar o tailsitter agora apenas para compartilhar PX4 com o tethered drone.

## Estrategias possiveis

### Estrategia A: usar o mesmo clone PX4 para tailsitter e tethered drone

Vantagens:

- economiza espaco local;
- uma unica instalacao PX4;
- atualizacoes centralizadas.

Desvantagens:

- pode conflitar com a estrutura autocontida do tailsitter;
- risco de uma branch de um projeto interferir no outro;
- exige disciplina forte de branches e paths.

Nao e a melhor escolha se o tailsitter ja esta organizado com PX4 interno.

### Estrategia B: novo fork/clone para o tethered drone

Vantagens:

- isolamento claro entre projetos;
- menor risco de quebrar o tailsitter;
- mais simples de entender;
- melhor para uma primeira migracao PX4 do tethered drone.

Desvantagens:

- ocupa mais espaco em disco;
- atualizacoes do PX4 precisam ser feitas separadamente;
- possivel duplicacao de ambiente.

Esta e a opcao mais pragmatica neste momento.

### Estrategia C: git worktree

Vantagens:

- permite varias branches em diretorios separados compartilhando o mesmo banco Git local;
- economiza espaco em relacao a varios clones completos;
- bom para manter branches como `tailsitter` e `tethered-drone`.

Exemplo:

```text
~/px4/PX4-Autopilot.git
~/px4/px4-tailsitter
~/px4/px4-tethered
```

Desvantagens:

- e mais avancado;
- pode ser confuso no inicio;
- nao encaixa tao bem se o tailsitter ja espera o PX4 dentro do diretorio do projeto.

Boa opcao futura, mas nao precisa ser a primeira.

### Estrategia D: submodule

Vantagens:

- o repositorio tethered drone pode apontar para um commit exato do PX4;
- melhora a reproducibilidade;
- evita copiar o PX4 inteiro para dentro do historico do projeto.

Desvantagens:

- submodules exigem cuidado;
- usuarios precisam rodar `git submodule update --init --recursive`;
- desenvolvimento simultaneo dentro do submodule pode confundir.

Pode ser uma boa opcao depois que a integracao PX4 estiver mais madura.

## Recomendacao pratica para este projeto

Como o tailsitter ja esta autocontido, a recomendacao e criar um fork proprio do PX4 e um clone separado para o tethered drone.

Estrutura sugerida:

```text
~/codes/ic/drone-cabo
~/codes/ic/PX4-Autopilot-tethered
```

No GitHub:

```text
seu_usuario/drone-cabo
seu_usuario/PX4-Autopilot
```

No clone local do PX4 para o tethered drone:

```bash
git remote add upstream https://github.com/PX4/PX4-Autopilot.git
git fetch upstream
```

Branches sugeridas no fork PX4:

```text
main-sync
tethered-drone
experimentos/tether-gazebo
```

O repositorio `drone-cabo` deve guardar:

- codigo ROS 2 especifico do tether;
- scripts de metricas;
- configuracoes de trajetorias;
- gerador do cabo;
- mundos e modelos especificos do tether;
- documentacao;
- referencia para o commit do PX4 usado.

O repositorio PX4 forkado deve guardar apenas:

- alteracoes realmente necessarias no PX4;
- modelo derivado do drone PX4, se nao for possivel mante-lo fora;
- configuracoes especificas que precisam viver dentro da arvore PX4.

## Como registrar a versao PX4 usada

No repositorio `drone-cabo`, criar futuramente um arquivo simples como:

```text
config/px4_version.txt
```

Conteudo sugerido:

```text
PX4 remote: https://github.com/seu_usuario/PX4-Autopilot.git
PX4 upstream: https://github.com/PX4/PX4-Autopilot.git
PX4 branch: tethered-drone
PX4 commit: <hash do commit>
Data de validacao: <data>
Teste validado: hover sem tether, hover com tether slack, N/S/E/W, etc.
```

Assim o projeto principal nao precisa conter o PX4 inteiro para ser reproduzivel. Ele precisa apenas documentar qual commit do PX4 foi usado.

## O que nao colocar no repositorio principal

Evitar colocar dentro do `drone-cabo`:

- clone completo do PX4;
- diretorios `build`, `install` e `log`;
- resultados grandes de simulacao;
- videos pesados;
- bags grandes do ROS 2;
- arquivos gerados que podem ser recriados.

Para resultados grandes, usar preferencialmente:

- releases do GitHub;
- Git LFS, se realmente necessario;
- armazenamento externo;
- uma pasta `results/` ignorada pelo Git, com apenas resumos versionados.

## Caminho incremental sugerido

1. Manter o branch `shared` como baseline atual do tether.
2. Criar fork PX4 na conta pessoal.
3. Clonar PX4 separadamente para o tethered drone.
4. Rodar PX4 SITL com um modelo padrao, sem tether.
5. Criar uma variante simples do modelo PX4 para expor o ponto de conexao.
6. Conectar o tether ao modelo PX4 no Gazebo.
7. Manter ROS 2 apenas lendo metricas inicialmente.
8. Comparar hover com tether slack entre arquitetura atual e PX4.
9. Migrar testes cardeais N/S/E/W.
10. Somente depois avaliar HIL/Pixhawk.

## Decisao final

Para este momento:

```text
Nao compartilhar imediatamente o PX4 interno do tailsitter.
Criar um fork/clone separado do PX4 para o tethered drone.
Manter o projeto drone-cabo como repositorio principal do tether, testes e documentacao.
Registrar explicitamente qual commit do PX4 foi usado em cada baseline.
```

Essa escolha sacrifica um pouco de espaco em disco, mas reduz bastante o risco de misturar dois projetos complexos.
