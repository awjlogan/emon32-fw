#!/bin/bash

cd .git/hooks
if [ ! -f pre-commit ]; then
    cat << 'EOF' > pre-commit
#!/bin/bash

clang-format -i src/*
clang-format -i tests/*.c
clang-format -i tests/*.h

EOF
chmod +x pre-commit
fi
