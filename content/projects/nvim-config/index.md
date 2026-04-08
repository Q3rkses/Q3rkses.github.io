---
title: "Neovim config for personal use"
date: 2026-01-01
draft: false
summary: "Neovim config based on Astrovim Distro, with some modifications, and some vibecoding functionality i use in day to day coding"
tags: ["Linux", "Arch", "Ubuntu", "Neovim", "dotfiles", "lua"]
---

<pre style="font-size: 0.45em; line-height: 1.2; letter-spacing: 0.05em; overflow-x: auto; text-align: center;">
███╗   ██╗███████╗ ██████╗ ██╗   ██╗██╗███╗   ███╗
████╗  ██║██╔════╝██╔═══██╗██║   ██║██║████╗ ████║
██╔██╗ ██║█████╗  ██║   ██║██║   ██║██║██╔████╔██║
██║╚██╗██║██╔══╝  ██║   ██║╚██╗ ██╔╝██║██║╚██╔╝██║
██║ ╚████║███████╗╚██████╔╝ ╚████╔╝ ██║██║ ╚═╝ ██║
╚═╝  ╚═══╝╚══════╝ ╚═════╝   ╚═══╝  ╚═╝╚═╝     ╚═╝
</pre>
 
<p style="text-align: center; opacity: 0.6; font-style: italic; margin-top: -0.5em;">
My editor is not an IDE. It's better.
</p>
 
---
 
## The Setup
 
A custom Neovim configuration built on the **AstroVim** template, managed with **lazy.nvim**. The config lives in a `lua/plugins/` folder structure and is designed around a productive development workflow with first-class Git integration via **diffview.nvim**.
 
---
 
## Cheatsheet
 
### Movement
 
| Key | Action |
|---|---|
| `w` | Next word start |
| `e` | End of word |
| `b` | Previous word |
| `0` | Start of line |
| `^` | First non-blank character |
| `$` | End of line |
| `<C-d>` / `<C-u>` | Half page down / up |
| `<C-f>` / `<C-b>` | Full page down / up |
| `zz` | Center cursor |
| `gg` / `G` | Top / bottom of file |
 
### Insert Mode Entry
 
| Key | Action |
|---|---|
| `i` / `a` | Insert before / after cursor |
| `I` / `A` | Insert at start / end of line |
| `o` / `O` | New line below / above + insert |
 
### Navigation Between Files
 
| Key | Action |
|---|---|
| `<leader>ff` | Find files |
| `<leader>fF` | Find all files (incl hidden/ignored) |
| `<leader>fg` | Find git-tracked files |
| `<leader>fw` | Find words across project |
| `<leader>fb` | Find open buffers |
| `<leader>fo` | Recent files |
| `<leader>bb` | Choose buffer from timeline |
| `<C-o>` / `<C-i>` | Jump back / forward |
 
### LSP
 
| Key | Action |
|---|---|
| `K` | Hover docs |
| `gd` | Go to definition |
| `gD` | Go to declaration |
| `gy` | Go to type definition |
| `gr` | List references |
 
### Editing Essentials
 
| Key | Action |
|---|---|
| `u` / `<C-r>` | Undo / redo |
| `v` / `V` | Character / line selection |
| `gv` | Reselect previous selection |
| `y` / `yy` | Yank / yank whole line |
| `d` / `x` | Cut / delete char under cursor |
| `p` / `P` | Paste after / before cursor |
| `>` / `<` | Indent / unindent |
| `.` | Repeat last change |
| `ciw` | Change inner word |
| `diw` | Delete inner word |
| `ci"` | Change inside quotes |
| `ci(` | Change inside parentheses |
| `ci{` | Change inside braces |
 
### File Tree
 
| Key | Action |
|---|---|
| `<leader>e` | Toggle file tree |
| `Enter` | Open file |
| `a` / `d` / `r` | Add / delete / rename |
| `m` / `y` / `p` | Move / copy / paste |
| `q` | Close tree |
| `<Tab>` / `<S-Tab>` | Next / previous source |
 
### Surround (mini.surround)
 
| Key | Action |
|---|---|
| `sa` | Add surrounding |
| `sd` | Delete surrounding |
| `sr` | Replace surrounding |
| `sf` / `sF` | Find surrounding right / left |
| `sh` | Highlight surrounding |
| `sn` | Update `n_lines` |
 
### Search & Replace
 
| Key | Action |
|---|---|
| `:%s/a/b/g` | Replace all in file |
| `:%s/a/b/gc` | Replace with confirmation |
 
---
 
### Merge Conflicts (diffview.nvim)
 
| Key | Action |
|---|---|
| `<leader>mo` | Choose OURS |
| `<leader>mt` | Choose THEIRS |
| `]x` / `[x` | Next / prev conflict (within file) |
| `<Tab>` / `<S-Tab>` | Cycle conflicted files |
 
```
Workflow: merge → :DiffviewOpen → navigate → resolve → save → :DiffviewClose → git add . && git merge --continue
```

## Links

- [AstroNeovim](https://astronvim.com)
- [Source code](https://github.com/Q3rkses/nvimconf)
