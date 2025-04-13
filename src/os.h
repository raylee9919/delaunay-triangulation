/* ========================================================================

   (C) Copyright 2025 by Sung Woo Lee, All Rights Reserved.

   This software is provided 'as-is', without any express or implied
   warranty. In no event will the authors be held liable for any damages
   arising from the use of this software.

   ======================================================================== */




void *os_alloc(size_t size) {
    void *result = VirtualAlloc(0, size, MEM_RESERVE|MEM_COMMIT, PAGE_READWRITE);
    ASSERT(result);
    return result;
}

void os_free(void *memory) {
    if (memory) VirtualFree(memory, 0, MEM_RELEASE);
}

