autocmd VimEnter * :packadd termdebug

let g:termdebug_wide = 163
let g:lsp_cxx_hl_use_text_props = 1

nmap <c-s-b> : make<CR>
nmap <F5> :Termdebug main<CR>
nmap <F6> :!./main<CR>
nmap <F7> : make<CR>
