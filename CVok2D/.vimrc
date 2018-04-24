let g:gtest#gtest_command = "../build/debug/unittest/unittest"
nnoremap <f10> :AsyncRun cmake --build ../build/debug
nnoremap <f7> :GTestRunUnderCursor <CR>
nnoremap <f8> :GTestRun
