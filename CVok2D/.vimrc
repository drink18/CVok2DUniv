let g:gtest#gtest_command = "../build/debug/unittest/unittest"
nnoremap <f10> :wa<bar>AsyncRun cmake --build ../build/debug <CR>
nnoremap <f7> :GTestRunUnderCursor <CR>
nnoremap <f8> :GTestRun
