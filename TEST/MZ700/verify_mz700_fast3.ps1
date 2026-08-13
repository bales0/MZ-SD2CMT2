$ErrorActionPreference = 'Stop'

$testRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $testRoot '..\..')
$romPath = Join-Path $testRoot 'mz700_1z-009a_jp.rom'
$sourcePath = Join-Path $repoRoot 'src\play\mz700_fast3.cpp'
$playbackPath = Join-Path $repoRoot 'src\play\mzf_playback.cpp'

function Assert-True([bool]$condition, [string]$message) {
    if (-not $condition) { throw $message }
}

function Read-Le16([byte[]]$bytes, [int]$offset) {
    return [int]$bytes[$offset] -bor ([int]$bytes[$offset + 1] -shl 8)
}

function Test-Overlap([int]$leftStart, [int]$leftLength,
                      [int]$rightStart, [int]$rightLength) {
    return ($leftStart -lt ($rightStart + $rightLength)) -and
           ($rightStart -lt ($leftStart + $leftLength))
}

function New-Stage([int]$runtimeAddress, [int]$runtimeSize) {
    if ($runtimeAddress -eq 0x1108) {
        return [byte[]]@(
            0xC3, 0x08, 0x11,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)
    }
    return [byte[]]@(
        0x21, 0x08, 0x11, 0x11, 0x00, 0xC0,
        0x01, ($runtimeSize -band 0xFF), (($runtimeSize -shr 8) -band 0xFF),
        0xED, 0xB0, 0xC3, 0x00, 0xC0)
}

$expectedRomHash = 'B0D16889AC3E2A80CC3BC9445BC95BC9988DF7B6115124F284850667CF45FF9F'
$actualRomHash = (Get-FileHash -LiteralPath $romPath -Algorithm SHA256).Hash
Assert-True ($actualRomHash -eq $expectedRomHash) 'Unexpected 1Z-009A ROM image.'

$rom = [IO.File]::ReadAllBytes($romPath)
Assert-True ($rom.Length -eq 4096) 'The monitor ROM must be exactly 4096 bytes.'
$qadcn = [byte[]]$rom[0x0A92..0x0B91]

# Ensure the firmware embeds the exact forward QADCN table from this ROM.
$source = Get-Content -Raw -LiteralPath $sourcePath
$tableMatch = [regex]::Match(
    $source,
    'mz700_qadcn_P\[256\].*?=\s*\{(?<body>.*?)\};',
    [Text.RegularExpressions.RegexOptions]::Singleline)
Assert-True $tableMatch.Success 'Firmware QADCN table was not found.'
$sourceTable = [regex]::Matches($tableMatch.Groups['body'].Value, '0x([0-9A-Fa-f]{2})U') |
    ForEach-Object { [Convert]::ToByte($_.Groups[1].Value, 16) }
Assert-True ($sourceTable.Count -eq 256) 'Firmware QADCN table must contain 256 bytes.'
for ($i = 0; $i -lt 256; ++$i) {
    Assert-True ($sourceTable[$i] -eq $qadcn[$i]) "QADCN mismatch at source byte $i."
}

# Every possible runtime size used by the implementation must encode and
# decode to the exact intended LOW/HIGH stage, without source byte $0D.
for ($runtimeSize = 86; $runtimeSize -le 103; ++$runtimeSize) {
    foreach ($runtimeAddress in @(0x1108, 0xC000)) {
        $stage = New-Stage $runtimeAddress $runtimeSize
        Assert-True ($stage.Count -eq 14) 'Stage 1 must remain exactly 14 bytes.'
        foreach ($opcode in $stage) {
            $encoded = -1
            for ($sourceByte = 0; $sourceByte -lt 256; ++$sourceByte) {
                if (($sourceByte -ne 0x0D) -and ($qadcn[$sourceByte] -eq $opcode)) {
                    $encoded = $sourceByte
                    break
                }
            }
            Assert-True ($encoded -ge 0) ("Opcode {0:X2} is not QADCN encodable." -f $opcode)
            Assert-True ($encoded -ne 0x0D) 'QMSGX terminator leaked into stage 1.'
            Assert-True ($qadcn[$encoded] -eq $opcode) 'Stage 1 round-trip failed.'
        }
    }
}

# Half-open overlap boundary regression checks.
$testRuntimeSize = 96
Assert-True (-not (Test-Overlap 0x1108 $testRuntimeSize 0x1107 1)) 'Byte ending at LOW start must not overlap.'
Assert-True (-not (Test-Overlap 0x1108 $testRuntimeSize 0x1168 1)) 'Byte starting after LOW end must not overlap.'
Assert-True (Test-Overlap 0x1108 $testRuntimeSize 0x1108 1) 'LOW start overlap was missed.'
Assert-True (Test-Overlap 0xC000 $testRuntimeSize 0xBFFF 2) 'HIGH crossing overlap was missed.'

# Bombman is the hardware-proven LOW reference.
$bombman = [IO.File]::ReadAllBytes((Join-Path $testRoot 'Bombman.mzf'))
$bombmanSize = Read-Le16 $bombman 0x12
$bombmanLoad = Read-Le16 $bombman 0x14
$bombmanExec = Read-Le16 $bombman 0x16
$bombmanName = [Text.Encoding]::ASCII.GetString($bombman, 1, 17).Trim([char]0, [char]13, ' ')
$bombmanRuntimeSize = 78 + 8 + $bombmanName.Length
Assert-True ($bombmanName -eq 'BOMBER MAN') 'Unexpected Bombman name.'
Assert-True ($bombmanSize -eq 0x2010) 'Unexpected Bombman SIZE.'
Assert-True ($bombmanLoad -eq 0x1200) 'Unexpected Bombman LOAD.'
Assert-True ($bombmanExec -eq 0x1200) 'Unexpected Bombman EXEC.'
Assert-True ($bombmanRuntimeSize -eq 96) 'Bombman runtime must be 96 bytes.'
Assert-True (-not (Test-Overlap 0x1108 $bombmanRuntimeSize $bombmanLoad $bombmanSize)) `
    'Bombman should select LOW.'

# A one-byte payload at LOW start must force HIGH; a large payload spanning
# both candidates must reject FAST3 and fall back safely.
Assert-True (Test-Overlap 0x1108 $testRuntimeSize 0x1108 1) 'Forced-HIGH fixture does not hit LOW.'
Assert-True (-not (Test-Overlap 0xC000 $testRuntimeSize 0x1108 1)) 'Forced-HIGH fixture unexpectedly hits HIGH.'
Assert-True ((Test-Overlap 0x1108 $testRuntimeSize 0x1000 0xB100) -and
             (Test-Overlap 0xC000 $testRuntimeSize 0x1000 0xB100)) 'Both-overlap fixture is invalid.'

# Lock the proven pulse constants and the only permitted ROM speed patch.
$playback = Get-Content -Raw -LiteralPath $playbackPath
foreach ($needle in @(
    'MZF_IC_1_3_SHORT_HIGH_TICKS MZF_US_TO_TICKS(112U)',
    'MZF_IC_1_3_SHORT_LOW_TICKS  MZF_US_TO_TICKS(96U)',
    'MZF_IC_1_3_LONG_HIGH_TICKS  MZF_US_TO_TICKS(224U)',
    'MZF_IC_1_3_LONG_LOW_TICKS   MZF_US_TO_TICKS(192U)')) {
    Assert-True $playback.Contains($needle) "Missing proven IC 1:3 timing: $needle"
}
Assert-True $playback.Contains(
    '#define MZF_MZ700_3X_SHORT_TICKS MZF_US_TO_TICKS(80U)') `
    'MZ700 FAST3 short symmetric interval is not 80 us.'
Assert-True $playback.Contains(
    '#define MZF_MZ700_3X_LONG_TICKS  MZF_US_TO_TICKS(160U)') `
    'MZ700 FAST3 long symmetric interval is not 160 us.'
$mz700ShortUses = ([regex]::Matches(
    $playback, 'return MZF_MZ700_3X_SHORT_TICKS;')).Count
$mz700LongUses = ([regex]::Matches(
    $playback, 'return MZF_MZ700_3X_LONG_TICKS;')).Count
Assert-True ($mz700ShortUses -eq 2) `
    'MZ700 FAST3 HIGH/LOW short phases are not symmetric.'
Assert-True ($mz700LongUses -eq 2) `
    'MZ700 FAST3 HIGH/LOW long phases are not symmetric.'
Assert-True $playback.Contains('OCR3B = (uint16_t)(OCR3B + ticks);') `
    'MZ700 FAST3 phases are not anchored to the previous Timer3 compare.'
Assert-True $playback.Contains('mzf_timer_start_phase_from_isr(low_ticks);') `
    'MZ700 FAST3 LOW phases still use the latency-adding timer restart.'
Assert-True $playback.Contains('mzf_timer_start_phase_from_isr(high_ticks);') `
    'MZ700 FAST3 HIGH phases still use the latency-adding timer restart.'
Assert-True $playback.Contains(
    'return mzf_stage_uses_tape_turbo_timing() && mzf_loader_is_ic_turbo();') `
    'MZ700 FAST3 must remain HIGH-first like the hardware-proven WAV.'
Assert-True $playback.Contains('#define MZF_MZ700_FAST3_START_DELAY_MS 400U') `
    'MZ700 FAST3 must preserve the proven 400 ms inter-block LOW gap.'
Assert-True (-not $playback.Contains('MZF_MZ700_FAST3_FINISH_DELAY_MS')) `
    'A timed FAST3 tail is redundant while READ/SENSE remain low until STOP.'
Assert-True $source.Contains('write_le16(destination + offset, 0x0A4BU)') `
    'The FAST3 runtime does not patch $0A4B.'
Assert-True (-not $source.Contains('0x0512U')) 'Forbidden $0512 patch detected.'

# The supplied Japanese 1Z-009A ROM has LD A,$45 / JP $0762 at DLY3.
# FAST3 replaces only that operand with $15.  RDDAT is entered through the
# public vector at $002A and its Carry error must return to monitor QER $00FE.
Assert-True (($rom[0x002A] -eq 0xC3) -and ($rom[0x002B] -eq 0xF8) -and
             ($rom[0x002C] -eq 0x04)) 'ROM $002A is not JP RDDAT ($04F8).'
Assert-True (($rom[0x0A4A] -eq 0x3E) -and ($rom[0x0A4B] -eq 0x45) -and
             ($rom[0x0A4C] -eq 0xC3) -and ($rom[0x0A4D] -eq 0x62) -and
             ($rom[0x0A4E] -eq 0x07)) 'Unexpected 1Z-009A DLY3 routine.'
Assert-True $source.Contains('destination[offset++] = 0x3EU; destination[offset++] = 0x15U;') `
    'FAST3 does not install the verified DLY3 operand $15.'
Assert-True $source.Contains('write_le16(destination + offset, 0x00FEU)') `
    'RDDAT Carry errors are not returned to monitor CMT ERROR handling.'
Assert-True $source.Contains('write_le16(destination + offset, 0xD027U)') `
    'The pre-RDDAT screen marker is missing.'
Assert-True (-not $source.Contains('stage[10] = 0xB8U')) `
    'LOW stage still contains the obsolete LDDR relocation.'

Write-Host 'MZ700 FAST3 verification passed.'
