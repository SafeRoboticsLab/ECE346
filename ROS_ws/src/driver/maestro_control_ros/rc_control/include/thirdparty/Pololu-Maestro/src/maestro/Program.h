#pragma once

#include <maestro/Instruction.h>

#include <cstdint>
#include <map>
#include <stack>
#include <string>
#include <vector>

namespace Maestro {

class Program {
   public:
    Program(const std::string& script, bool isMiniMaestro);

    std::vector<uint8_t> getByteList() const;
    uint16_t getCRC() const;
    std::string toString() const;

   private:
    enum class BlockType { BEGIN = 0, IF, ELSE };
    enum class Mode { NORMAL, GOTO, SUBROUTINE };

    void addLiteral(int literal, const std::string& filename, int lineNumber, int columnNumber, bool isMiniMaestro);

    void openBlock(BlockType blocktype, const std::string& filename, int line_number, int column_number);
    BlockType getCurrentBlockType() const;
    std::string getCurrentBlockStartLabel() const;
    std::string getCurrentBlockEndLabel() const;
    std::string getNextBlockEndLabel() const;

    void closeBlock(const std::string& filename, int line_number, int column_number);
    void completeJumps();
    void completeCalls(bool isMiniMaestro);
    void completeLiterals();

    void parseGoto(std::string s, std::string filename, int line_number, int column_number, Mode& mode);
    void parseSubroutine(std::string s, std::string filename, int line_number, int column_number, Mode& mode);
    bool looksLikeLiteral(const std::string& s);
    void parseString(std::string s, std::string filename, int line_number, int column_number, bool isMiniMaestro, Mode& mode);

    Instruction& findLabel(const std::string& name);

    std::vector<std::string> m_sourceLines;
    std::vector<Instruction> m_instructionList;
    std::map<std::string, uint16_t> m_subroutineAddresses;
    std::map<std::string, Opcode> m_subroutineCommands;
    int m_maxBlock = 0;
    std::stack<int> m_openBlocks;
    std::stack<BlockType> m_openBlockTypes;
};
}  // namespace Maestro
