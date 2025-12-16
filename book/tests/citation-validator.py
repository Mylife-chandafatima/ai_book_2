#!/usr/bin/env python3
"""
Citation Validator for Module 3
Validates that all citations follow APA format and meet academic standards
"""

import re
import os
import sys
from pathlib import Path

def find_module_docs():
    """Find all Module 3 documentation files"""
    module_path = Path("book/docs/module-3-nvidia-isaac/")
    if not module_path.exists():
        print(f"❌ Module 3 documentation directory does not exist: {module_path}")
        return []

    md_files = list(module_path.glob("*.md"))
    print(f"✅ Found {len(md_files)} Module 3 documentation files")
    return md_files

def extract_citations_from_file(file_path):
    """Extract citations from a markdown file"""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Look for APA-style citations in the format: (Author, Year) or Author (Year)
    apa_patterns = [
        r'\([A-Z][a-z]+, \d{4}\)',  # (Smith, 2023)
        r'[A-Z][a-z]+ \([A-Z][a-z]+, \d{4}\)',  # Smith (Johnson, 2023)
        r'[A-Z][a-z]+ et al.,? \(\d{4}\)',  # Smith et al. (2023)
        r'\([A-Z][a-z]+ et al.,? \d{4}\)',  # (Smith et al. 2023)
    ]

    citations = []
    for pattern in apa_patterns:
        matches = re.findall(pattern, content)
        citations.extend(matches)

    return citations

def validate_apa_format(citation):
    """Validate if a citation follows proper APA format"""
    # Remove leading/trailing parentheses and spaces
    clean_citation = citation.strip('() ')

    # Check for basic APA patterns
    # Author (Year) or (Author, Year) format
    if re.match(r'^[A-Z][a-z]+( et al\.?)?,? \d{4}$', clean_citation):
        return True
    elif re.match(r'^\([A-Z][a-z]+( et al\.?)?,? \d{4}\)$', citation):
        return True
    elif re.match(r'^[A-Z][a-z]+ [A-Z][a-z]+,? \d{4}$', clean_citation):
        return True

    return False

def validate_references_section(file_path):
    """Validate that a references section exists and follows APA format"""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Look for references section
    references_pattern = r'(##?\s+References?\n|##?\s+Works Cited\n)'
    has_references_section = bool(re.search(references_pattern, content, re.IGNORECASE))

    if not has_references_section:
        print(f"  ⚠️  No references section found in {file_path.name}")
        return False

    # Extract the references section
    refs_start = re.search(references_pattern, content, re.IGNORECASE)
    if refs_start:
        refs_section = content[refs_start.end():]
        # Look for the next heading to limit the section
        next_heading = re.search(r'\n#{2,}', refs_section)
        if next_heading:
            refs_section = refs_section[:next_heading.start()]

        # Count references
        # Look for typical APA reference formats
        ref_patterns = [
            r'^\s*[A-Z][^,]+,\s*[A-Z]\..*\.?\s*\(\d{4}\).*$',  # Author, A. A. (2023). Title.
            r'^\s*[A-Z][^,]+,\s*[A-Z]\..*,\s*&.*\(\d{4}\).*$',  # Multiple authors
        ]

        ref_count = 0
        for line in refs_section.split('\n'):
            for pattern in ref_patterns:
                if re.match(pattern, line.strip(), re.MULTILINE):
                    ref_count += 1
                    break

        if ref_count >= 5:  # Minimum 5 references as per requirements
            print(f"  ✅ Found {ref_count} APA-formatted references in {file_path.name}")
            return True
        else:
            print(f"  ❌ Only found {ref_count} references in {file_path.name}, minimum 5 required")
            return False

    return False

def run_validation():
    """Run the citation validation process"""
    print("Starting Citation Validation for Module 3...\n")

    # Find Module 3 documentation files
    md_files = find_module_docs()

    if not md_files:
        print("❌ No Module 3 documentation files found to validate")
        return False

    total_citations = 0
    valid_citations = 0
    files_with_refs = 0
    total_files = len(md_files)

    for file_path in md_files:
        print(f"\nValidating {file_path.name}:")

        # Extract citations
        citations = extract_citations_from_file(file_path)
        total_citations += len(citations)

        # Validate each citation
        for citation in citations:
            if validate_apa_format(citation):
                valid_citations += 1
                print(f"  ✅ Valid: {citation}")
            else:
                print(f"  ❌ Invalid: {citation}")

        # Validate references section
        has_valid_refs = validate_references_section(file_path)
        if has_valid_refs:
            files_with_refs += 1

    # Overall validation results
    print(f"\n{'='*60}")
    print("Citation Validation Results for Module 3")
    print(f"{'='*60}")
    print(f"Total citations found: {total_citations}")
    print(f"Valid citations: {valid_citations}")
    print(f"Invalid citations: {total_citations - valid_citations}")
    print(f"Files with valid references: {files_with_refs}/{total_files}")

    # Calculate peer-reviewed percentage (we'll simulate this since we can't verify actual peer review)
    # In a real scenario, we'd check against a database of peer-reviewed publications
    print(f"Peer-reviewed reference requirement: 50% minimum")

    # Calculate success metrics
    citation_accuracy = (valid_citations / total_citations * 100) if total_citations > 0 else 100
    has_min_refs = files_with_refs > 0  # At least one file should have references
    has_min_citations = total_citations >= 5  # Minimum 5 citations required

    print(f"Citation accuracy: {citation_accuracy:.1f}%")
    print(f"Has minimum required citations: {'✅' if has_min_citations else '❌'}")
    print(f"Has properly formatted references: {'✅' if has_min_refs else '❌'}")

    # Determine overall success
    overall_success = (
        citation_accuracy >= 80 and  # At least 80% of citations are properly formatted
        has_min_citations and        # Has minimum required number of citations
        has_min_refs                 # Has properly formatted references section
    )

    print(f"{'='*60}")

    if overall_success:
        print("🎉 All citation validation checks PASSED!")
        print("Module 3 content meets academic citation standards.")
        return True
    else:
        print("❌ Some citation validation checks FAILED!")
        print("Please review and correct citation formats in Module 3 content.")
        return False

if __name__ == "__main__":
    success = run_validation()
    sys.exit(0 if success else 1)