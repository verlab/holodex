# Guia de contribuição — Git e fluxo de trabalho

Este documento descreve como colaborar neste repositório VerLab: branches, commits, pull requests e boas práticas.

## Branches principais

| Branch | Uso |
|--------|-----|
| `main` | Código estável, revisado e pronto para release / demonstração |
| `dev` | Integração contínua — merge das features antes de ir para `main` |

**Regra:** não commite diretamente em `main` ou `dev`. Abra sempre uma branch de trabalho e integre via **Pull Request**.

## Branches de trabalho

Crie a branch a partir de `dev` (salvo hotfix urgente em produção, discutido com o time):

```bash
git fetch origin
git checkout dev
git pull origin dev
git checkout -b <tipo>/<descricao-curta>
```

### Convenção de nomes

| Prefixo | Quando usar | Exemplo |
|---------|-------------|---------|
| `feature/` | Nova funcionalidade ou módulo | `feature/holonomic-kinematics` |
| `fix/` | Correção de bug | `fix/wheel-slip-compensation` |
| `docs/` | Apenas documentação | `docs/nema23-wiring` |
| `chore/` | Manutenção (CI, deps, formatação) | `chore/update-ros-deps` |
| `refactor/` | Refatoração sem mudar comportamento | `refactor/odometry-pipeline` |
| `test/` | Testes novos ou ajustes de teste | `test/add-kinematics-unit` |
| `hotfix/` | Correção crítica em `main` (raro) | `hotfix/motor-estop` |

Use **kebab-case** (minúsculas, hífens), sem espaços. Seja específico: `feature/wheels` é fraco; `feature/omni-wheel-stl-v2` é melhor.

## Commits

### Mensagens (Conventional Commits)

```
<tipo>(<escopo opcional>): <descrição imperativa curta>

[corpo opcional — o que e por quê, não o como linha a linha]

[rodapé opcional — refs a issues]
```

**Tipos comuns:** `feat`, `fix`, `docs`, `chore`, `refactor`, `test`, `build`, `ci`, `perf`.

**Exemplos:**

```
feat(control): add holonomic velocity controller
fix(hardware): correct NEMA23 microstepping config
docs: add 3D print orientation guide for omni wheels
chore(ci): add colcon build workflow
```

### Boas práticas

- Um commit = uma mudança lógica coerente (fácil de reverter e revisar).
- Descrição no **imperativo** (“add sensor”, não “added sensor”).
- Não misture refatoração grande com feature no mesmo commit.
- Evite commits com arquivos gerados, builds (`build/`, `install/`) ou segredos (`.env`, chaves, tokens).
- Antes de push: `git status` limpo do que não deve ir; revise o diff (`git diff`).

### Commits pequenos vs. WIP

- Prefira commits que compilam / passam nos testes do escopo alterado.
- Evite mensagens genéricas: `fix`, `update`, `wip`, `aaa`.
- Se precisar de WIP temporário, squash antes do PR (rebase interativo ou “Squash and merge” no GitHub).

## Pull Requests

### Fluxo padrão

```
feature/fix branch  →  dev  →  main
```

1. Abra PR da sua branch **para `dev`**.
2. Após revisão e CI (quando existir), merge em `dev`.
3. Quando `dev` estiver estável, abra PR **`dev` → `main`** (release / marco).

Exceção: `hotfix/*` pode ir direto para `main` com PR dedicado; depois faça **backport** para `dev` (merge ou cherry-pick).

### Título do PR

Use o mesmo estilo dos commits:

```
feat(control): holonomic inverse kinematics node
fix(mechanics): omni wheel roller diameter in URDF
```

### Descrição sugerida

```markdown
## Resumo
- O que mudou e por quê (1–3 frases)

## Como testar
- [ ] Passos para reproduzir / validar no hardware ou sim

## Checklist
- [ ] Branch atualizada com a base (`dev`)
- [ ] Sem arquivos sensíveis ou artefatos de build
- [ ] Documentação atualizada (se aplicável)
```

### Revisão

- Responda comentários no PR; não force-push em branch compartilhada sem avisar.
- Prefira **“Squash and merge”** em features pequenas para histórico limpo em `dev`.
- Use **merge commit** ou rebase conforme combinado pelo time quando o PR for grande e bem segmentado.

### Atualizar branch com a base

```bash
git fetch origin
git checkout minha-branch
git rebase origin/dev   # ou: git merge origin/dev
git push --force-with-lease   # só se já tinha push e fez rebase
```

Prefira `--force-with-lease` a `--force` para não sobrescrever trabalho alheio.

## O que evitar

| Não faça | Motivo |
|----------|--------|
| Push direto em `main` / `dev` | Sem revisão, histórico instável |
| `git push --force` em `main` ou `dev` | Perda de histórico para todo o time |
| Commits gigantes “misc changes” | Revisão impossível, revert difícil |
| Segredos no repositório | Use variáveis de ambiente / secrets do GitHub |
| Binários pesados sem necessidade | Use releases, LFS ou assets externos |

## Issues (opcional)

Associe PRs a issues quando existirem:

```
Closes #12
Refs #8
```

## Referência rápida

```bash
# Nova feature
git checkout dev && git pull
git checkout -b feature/minha-feature
# ... editar, testar ...
git add -p                    # revisar hunks
git commit -m "feat(scope): descrição"
git push -u origin feature/minha-feature
# Abrir PR no GitHub: base = dev
```

Dúvidas sobre processo: alinhe com os mantenedores do repositório antes de mudar o fluxo em `main`.
